#!/usr/bin/env python3
"""
gw_config_tool.py - upload WiFi and ThingsBoard settings to the ESP32 over USB.

WHAT IT IS FOR
    A gateway in a cabinet needs its site WiFi credentials and its ThingsBoard
    access token, and the person who has those is rarely the person with the
    toolchain. This writes them into the ESP32's NVS over the same USB cable
    used to flash it, so the firmware binary is identical on every unit and
    identity arrives at commissioning time.

REQUIREMENTS
    Python 3.8+ and pyserial.

        pip install pyserial

    If PlatformIO is installed there is already a Python with pyserial in it:

        C:\\Users\\<you>\\.platformio\\penv\\Scripts\\python.exe tools/gw_config_tool.py

USAGE
    python tools/gw_config_tool.py            opens the window
    python tools/gw_config_tool.py --port COM15 --read
    python tools/gw_config_tool.py --port COM15 --ssid Plant --pass s3cret \\
        --host demo.thingsboard.io --token A1_TEST_TOKEN --write --reboot

    The command line exists so a production line can flash and commission a
    batch from a script. The window is for one unit at a time.

THE DEVICE END
    IoTHandler/src/provisioning.cpp, reached in the `tbgateway` build. Every
    reply line from it starts with '+', which is how this tool tells answers
    apart from the running log on the same port. Anything not starting with '+'
    is device log output and is shown in the lower pane - that log is usually
    where the reason for a failed connection actually appears, so it is not
    hidden.
"""

import argparse
import queue
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.stderr.write(
        "pyserial is required:  pip install pyserial\n"
        "or run this with PlatformIO's interpreter:\n"
        r"  %USERPROFILE%\.platformio\penv\Scripts\python.exe tools/gw_config_tool.py" "\n"
    )
    raise SystemExit(2)


BAUD = 115200
REPLY_TIMEOUT_S = 4.0

# Field order is the order they appear in the window. Each entry is
# (nvs key, label, kind), where kind drives the widget and the validation.
FIELDS = [
    ("wifi.ssid", "WiFi SSID", "text"),
    ("wifi.pass", "WiFi password", "password"),
    ("tb.host", "ThingsBoard host", "text"),
    ("tb.port", "Port", "int"),
    ("tb.token", "Device access token", "text"),
    ("tb.tls", "Use TLS", "bool"),
    ("dev.name", "Device name", "text"),
    ("tb.telemetryMs", "Telemetry period (ms)", "int"),
]


class GatewayLink:
    """One serial connection to a gateway, speaking the '+' line protocol."""

    def __init__(self, port, log=None):
        self.port = port
        self._log = log or (lambda _line: None)
        self._ser = None
        self._rx = ""

    def open(self):
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = BAUD
        ser.timeout = 0.15
        # Leave DTR and RTS alone. On this board they are wired to EN and IO0,
        # so asserting them the way pyserial does by default would reset the
        # ESP32 - and a tool that reboots the device just by connecting to it
        # makes "did my settings stick" impossible to answer.
        ser.dtr = False
        ser.rts = False
        ser.open()
        self._ser = ser
        # Discard whatever log output was already in flight.
        time.sleep(0.2)
        ser.reset_input_buffer()
        self._rx = ""

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    def _read_lines(self, ser, deadline):
        """Yields whole lines until the deadline, log lines included."""
        while time.time() < deadline:
            chunk = ser.read(512)
            if chunk:
                self._rx += chunk.decode("utf-8", "replace")
                while "\n" in self._rx:
                    line, self._rx = self._rx.split("\n", 1)
                    yield line.rstrip("\r")
            else:
                yield None  # nothing arrived in this slice - let the caller idle

    def command(self, text):
        """
        Sends one command and collects its reply.

        Returns (ok, values, error) where `values` is a dict of every
        `+key=value` line in the reply. Raises IOError only when the port dies;
        a device that answers +ERR is a normal, reported outcome.
        """
        ser = self._ser
        if ser is None or not ser.is_open:
            raise IOError("not connected")

        ser.reset_input_buffer()
        self._rx = ""
        ser.write((text + "\n").encode("utf-8"))
        ser.flush()

        values = {}
        deadline = time.time() + REPLY_TIMEOUT_S

        for line in self._read_lines(ser, deadline):
            if line is None:
                continue
            if not line.startswith("+"):
                # Device log. Worth showing - it explains most failures.
                if line.strip():
                    self._log(line)
                continue

            body = line[1:]
            if body == "OK":
                return True, values, None
            if body.startswith("ERR"):
                return False, values, body[3:].strip() or "device refused"
            if body.startswith("GW "):
                values["_identity"] = body[3:].strip()
                continue
            if "=" in body:
                k, v = body.split("=", 1)
                values[k] = v

        return False, values, "timed out waiting for +OK"


def list_ports():
    """Candidate ports, most-likely-first. Bluetooth links are filtered out."""
    out = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "bluetooth" in desc:
            continue
        out.append((p.device, p.description or ""))
    return out


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

def run_cli(args):
    link = GatewayLink(args.port, log=lambda s: print("   [dev]", s))
    try:
        link.open()
    except Exception as exc:
        print("cannot open %s: %s" % (args.port, exc))
        return 2

    try:
        ok, values, err = link.command("GW?")
        if not ok:
            print("no gateway on %s: %s" % (args.port, err))
            print("is it flashed with the 'tbgateway' environment?")
            return 1
        print("device:", values.get("_identity", "?"))

        writes = [
            ("wifi.ssid", args.ssid),
            ("wifi.pass", getattr(args, "password")),
            ("tb.host", args.host),
            ("tb.port", str(args.tb_port) if args.tb_port else None),
            ("tb.token", args.token),
            ("dev.name", args.name),
            ("tb.tls", "1" if args.tls else None),
        ]
        writes = [(k, v) for k, v in writes if v is not None]

        if args.write:
            if not writes:
                print("--write given but no fields to set")
                return 2
            for key, value in writes:
                ok, _, err = link.command("SET %s %s" % (key, value))
                shown = "<hidden>" if key == "wifi.pass" else value
                print("  SET %-16s %-28s %s"
                      % (key, shown, "ok" if ok else "FAILED: " + (err or "no reply")))
                if not ok:
                    return 1
            ok, _, err = link.command("SAVE")
            print("  SAVE %s" % ("ok" if ok else "FAILED: " + (err or "no reply")))
            if not ok:
                return 1

        if args.read or args.write:
            ok, values, err = link.command("GET")
            if ok:
                print("stored configuration:")
                for key, _, _ in FIELDS:
                    print("  %-16s %s" % (key, values.get(key, "")))
                print("  %-16s %s" % ("provisioned", values.get("provisioned", "?")))
            else:
                print("GET failed:", err)

        if args.status:
            ok, values, err = link.command("STATUS")
            if ok:
                print("status:")
                for k in sorted(values):
                    if not k.startswith("_"):
                        print("  %-20s %s" % (k, values[k]))
            else:
                print("STATUS failed:", err)

        if args.reboot:
            ok, _, err = link.command("REBOOT")
            print("REBOOT %s" % ("ok" if ok else "FAILED: " + (err or "no reply")))

        return 0
    finally:
        link.close()


# ---------------------------------------------------------------------------
# Window
# ---------------------------------------------------------------------------

def run_gui(initial_port=None):
    import tkinter as tk
    from tkinter import messagebox, ttk

    root = tk.Tk()
    root.title("IIoT Gateway - cloud configuration")
    root.minsize(640, 560)

    link = GatewayLink("")
    log_queue = queue.Queue()
    link._log = log_queue.put

    vars_by_key = {}

    # ---- layout -----------------------------------------------------------
    outer = ttk.Frame(root, padding=12)
    outer.pack(fill="both", expand=True)

    conn = ttk.LabelFrame(outer, text="Connection", padding=10)
    conn.pack(fill="x")

    ttk.Label(conn, text="Serial port").grid(row=0, column=0, sticky="w", padx=(0, 8))
    port_var = tk.StringVar()
    port_box = ttk.Combobox(conn, textvariable=port_var, width=34, state="readonly")
    port_box.grid(row=0, column=1, sticky="we")
    conn.columnconfigure(1, weight=1)

    def refresh_ports():
        ports = list_ports()
        port_box["values"] = ["%s  -  %s" % (d, desc) for d, desc in ports]
        if ports and not port_var.get():
            port_var.set(port_box["values"][0])

    ttk.Button(conn, text="Rescan", command=refresh_ports, width=10).grid(row=0, column=2, padx=6)

    status_var = tk.StringVar(value="not connected")
    ttk.Label(conn, textvariable=status_var, foreground="#666").grid(
        row=1, column=0, columnspan=3, sticky="w", pady=(8, 0)
    )

    form = ttk.LabelFrame(outer, text="Cloud settings", padding=10)
    form.pack(fill="x", pady=(12, 0))
    form.columnconfigure(1, weight=1)

    for row, (key, label, kind) in enumerate(FIELDS):
        ttk.Label(form, text=label).grid(row=row, column=0, sticky="w", pady=3, padx=(0, 10))
        if kind == "bool":
            var = tk.BooleanVar(value=False)
            ttk.Checkbutton(form, variable=var).grid(row=row, column=1, sticky="w", pady=3)
        else:
            var = tk.StringVar()
            entry = ttk.Entry(form, textvariable=var, width=44)
            if kind == "password":
                entry.configure(show="*")
            entry.grid(row=row, column=1, sticky="we", pady=3)
        vars_by_key[key] = var

    ttk.Label(
        form,
        text=("The password is never read back from the device.\n"
              "Leave it blank to keep the one already stored."),
        foreground="#666",
        justify="left",
    ).grid(row=len(FIELDS), column=1, sticky="w", pady=(6, 0))

    btns = ttk.Frame(outer)
    btns.pack(fill="x", pady=(12, 0))

    log_frame = ttk.LabelFrame(outer, text="Device output", padding=6)
    log_frame.pack(fill="both", expand=True, pady=(12, 0))
    log_text = tk.Text(log_frame, height=12, wrap="none", font=("Consolas", 9))
    log_text.pack(side="left", fill="both", expand=True)
    scroll = ttk.Scrollbar(log_frame, command=log_text.yview)
    scroll.pack(side="right", fill="y")
    log_text.configure(yscrollcommand=scroll.set)

    def log(msg):
        log_text.insert("end", msg + "\n")
        log_text.see("end")

    # ---- actions ----------------------------------------------------------
    def selected_port():
        raw = port_var.get()
        return raw.split(" ")[0] if raw else ""

    def set_connected(connected, detail=""):
        status_var.set(detail if detail else ("connected" if connected else "not connected"))
        state = "normal" if connected else "disabled"
        for b in (btn_read, btn_write, btn_status, btn_reboot):
            b.configure(state=state)
        btn_connect.configure(text="Disconnect" if connected else "Connect")

    def do_connect():
        if link.is_open:
            link.close()
            set_connected(False)
            log("disconnected")
            return

        port = selected_port()
        if not port:
            messagebox.showwarning("No port", "Pick a serial port first.")
            return
        link.port = port
        try:
            link.open()
        except Exception as exc:
            messagebox.showerror("Cannot open port", str(exc))
            return

        ok, values, err = link.command("GW?")
        if not ok:
            # A port that opens but does not answer is nearly always the wrong
            # firmware, so say that rather than just "timeout".
            link.close()
            messagebox.showerror(
                "No gateway found",
                "%s opened, but nothing answered GW?.\n\n"
                "Is this board flashed with the 'tbgateway' environment?\n"
                "  pio run -e tbgateway -t upload" % port,
            )
            log("GW? failed on %s: %s" % (port, err))
            return

        identity = values.get("_identity", "")
        set_connected(True, "connected to %s  -  %s" % (port, identity))
        log("connected: " + identity)
        do_read()

    def do_read():
        ok, values, err = link.command("GET")
        if not ok:
            messagebox.showerror("Read failed", err or "no reply")
            return
        for key, _, kind in FIELDS:
            if key == "wifi.pass":
                continue  # never sent back; leaving it blank means "keep"
            raw = values.get(key, "")
            if kind == "bool":
                vars_by_key[key].set(raw == "1")
            else:
                vars_by_key[key].set(raw)
        log("read configuration (%s)" % ("provisioned" if values.get("provisioned") == "1"
                                         else "NOT provisioned"))

    def do_write():
        ssid = vars_by_key["wifi.ssid"].get().strip()
        token = vars_by_key["tb.token"].get().strip()
        host = vars_by_key["tb.host"].get().strip()

        missing = [n for n, v in (("WiFi SSID", ssid), ("host", host), ("token", token)) if not v]
        if missing:
            messagebox.showwarning(
                "Incomplete",
                "These are required before the gateway can connect:\n  " + "\n  ".join(missing),
            )
            return

        if vars_by_key["tb.tls"].get():
            # Said plainly, once, at the moment it is chosen. The firmware does
            # not validate the server certificate yet, so TLS here encrypts the
            # link but does not prove who is on the other end of it.
            if not messagebox.askokcancel(
                "TLS without certificate validation",
                "This firmware enables TLS but does not verify the server's\n"
                "certificate. Traffic is encrypted, but an intercepting server\n"
                "could present its own certificate and collect the access token.\n\n"
                "Use it on a trusted network, or add a CA to tb_client.cpp first.\n\n"
                "Continue?",
            ):
                return

        for key, _, kind in FIELDS:
            var = vars_by_key[key]
            if kind == "bool":
                value = "1" if var.get() else "0"
            else:
                value = var.get().strip()

            if key == "wifi.pass" and value == "":
                log("  wifi.pass left unchanged")
                continue
            if kind != "bool" and value == "" and key != "wifi.pass":
                continue

            ok, _, err = link.command("SET %s %s" % (key, value))
            shown = "<hidden>" if key == "wifi.pass" else value
            log("  SET %-16s %-24s %s" % (key, shown, "ok" if ok else "FAILED: " + (err or "")))
            if not ok:
                messagebox.showerror("Write failed", "%s: %s" % (key, err))
                return

        ok, _, err = link.command("SAVE")
        if not ok:
            messagebox.showerror("Save failed", err or "no reply")
            return
        log("saved to NVS")

        if messagebox.askyesno(
            "Saved", "Configuration written.\n\nReboot the gateway now so it connects?"
        ):
            do_reboot()

    def do_status():
        ok, values, err = link.command("STATUS")
        if not ok:
            messagebox.showerror("Status failed", err or "no reply")
            return
        log("--- status ---")
        for k in sorted(values):
            if not k.startswith("_"):
                log("  %-20s %s" % (k, values[k]))

    def do_reboot():
        ok, _, err = link.command("REBOOT")
        log("reboot: " + ("ok" if ok else "FAILED: " + (err or "")))
        link.close()
        set_connected(False)

    btn_connect = ttk.Button(btns, text="Connect", command=do_connect, width=13)
    btn_connect.pack(side="left")
    btn_read = ttk.Button(btns, text="Read", command=do_read, width=11, state="disabled")
    btn_read.pack(side="left", padx=6)
    btn_write = ttk.Button(btns, text="Write + Save", command=do_write, width=14, state="disabled")
    btn_write.pack(side="left", padx=6)
    btn_status = ttk.Button(btns, text="Status", command=do_status, width=11, state="disabled")
    btn_status.pack(side="left", padx=6)
    btn_reboot = ttk.Button(btns, text="Reboot", command=do_reboot, width=11, state="disabled")
    btn_reboot.pack(side="left", padx=6)

    def drain_log():
        try:
            while True:
                log("   [dev] " + log_queue.get_nowait())
        except queue.Empty:
            pass
        root.after(120, drain_log)

    refresh_ports()
    if initial_port:
        for value in port_box["values"]:
            if value.startswith(initial_port):
                port_var.set(value)
                break
    drain_log()

    def on_close():
        link.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Upload WiFi and ThingsBoard settings to the ESP32 gateway.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", help="serial port, e.g. COM15 or /dev/ttyUSB0")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--ssid")
    ap.add_argument("--pass", dest="password", help="WiFi passphrase")
    ap.add_argument("--host", help="ThingsBoard host")
    ap.add_argument("--tb-port", type=int, help="MQTT port (1883 plain, 8883 TLS)")
    ap.add_argument("--token", help="device access token")
    ap.add_argument("--name", help="device name / MQTT client id")
    ap.add_argument("--tls", action="store_true", help="enable TLS")
    ap.add_argument("--read", action="store_true", help="print the stored configuration")
    ap.add_argument("--write", action="store_true", help="write the given fields, then SAVE")
    ap.add_argument("--status", action="store_true", help="print live status")
    ap.add_argument("--reboot", action="store_true", help="reboot when done")
    args = ap.parse_args()

    if args.list:
        ports = list_ports()
        for device, desc in ports:
            print("%-10s %s" % (device, desc))
        if not ports:
            # An empty list looks like the tool failed. Say what was looked for
            # and what was deliberately left out.
            print("no serial ports found (Bluetooth serial links are hidden).")
            print("check the USB cable, and that the board is not held in "
                  "download mode.")
        return 0

    headless = any([args.read, args.write, args.status, args.reboot])
    if headless:
        if not args.port:
            ap.error("--port is required for command-line use")
        return run_cli(args)

    return run_gui(args.port)


if __name__ == "__main__":
    raise SystemExit(main())
