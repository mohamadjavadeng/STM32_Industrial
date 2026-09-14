# Commissioning the gateway: relays, inputs, ThingsBoard

Four relays and four digital inputs on the STM32, published to ThingsBoard and
switched from a dashboard. This is the whole path, end to end.

Companion documents: `GATEWAY_ARCHITECTURE.md` (why the chips are split),
`shared/gw_model.h` (the wire contract), `LINK_V2_TESTING.html` (bring-up).

---

## 1. Wiring

| Terminal | STM32 pin | Mode | Notes |
|----------|-----------|------|-------|
| Relay 1 | PD0 | Output, push-pull | **Active LOW** by default |
| Relay 2 | PD1 | Output, push-pull | |
| Relay 3 | PD2 | Output, push-pull | |
| Relay 4 | PD3 | Output, push-pull | |
| Input 1 | PD4 | Input, internal pull-up | Closed contact to GND reads ON |
| Input 2 | PD5 | Input, internal pull-up | |
| Input 3 | PD6 | Input, internal pull-up | |
| Input 4 | PD7 | Input, internal pull-up | |

Both polarities are single constants in `ModbusRTU_AWS/Core/Inc/gw_link_cfg.h`:

```c
#define GW_LIO_DO_ACTIVE_LOW 1   /* 0 for a MOSFET/SSR board that drives HIGH */
#define GW_LIO_DI_ACTIVE_LOW 1   /* 0 for a source-type sensor driving HIGH   */
```

They are the only place the *inversion* exists. Everything above the driver —
the process image, the link, MQTT, the dashboard — deals in logical ON and OFF,
so a wrong guess about the relay board costs one character rather than a hunt
across four components.

The pins themselves are configured by `MX_GPIO_Init()`, generated from
`ModbusRTU_AWS.ioc`, which is this project's source of truth for peripheral
init:

| `.ioc` entry | Value |
|--------------|-------|
| `PD0..PD3.Signal` | `GPIO_Output` |
| `PD0..PD3.PinState` | `GPIO_PIN_SET` — idle high, de-energized for an active-low board |
| `PD4..PD7.Signal` | `GPIO_Input` |
| `PD4..PD7.GPIO_PuPd` | `GPIO_PULLUP` |

**The idle level is in two places and they must agree.** `MX_GPIO_Init` runs
long before the driver does, and the outputs have to be safe in between, so the
de-energized level is baked into the `.ioc` as well:

```
GW_LIO_DO_ACTIVE_LOW 1  ->  .ioc PinState = GPIO_PIN_SET    (idle high)
GW_LIO_DO_ACTIVE_LOW 0  ->  .ioc PinState = GPIO_PIN_RESET  (idle low)
```

Change one without the other and all four relays energize for the few
milliseconds between `MX_GPIO_Init` and `GwLocalIO_Init`. On a bench that is an
audible click; on a machine it is four contactors closing at power-up.

The ESP32↔STM32 link is unchanged: ESP32 GPIO18 ← PB10, GPIO17 → PB11, common
ground mandatory.

### Inputs are not isolated

PD4–PD7 go straight into the MCU with only its internal pull-up (roughly 40 kΩ).
That is fine for a dry contact or a pushbutton a short distance away in the same
cabinet. It is not fine for anything that leaves the enclosure, carries 24 V, or
runs alongside motor cable — those need an opto-isolator and a series resistor in
front of the pin. Debounce in firmware does not protect the pin from a
transient; it only cleans up a signal the pin survived.

---

## 2. Build and flash

### STM32 (industrial half)

`GW_LINK_ENABLE` and `GW_LOCALIO_ENABLE` are both 1 by default.

```sh
# Through the IDE, or headless:
"C:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/headless-build.bat" \
    -data <a scratch workspace dir> -import ModbusRTU_AWS -build ModbusRTU_AWS/Debug
```

Two things worth knowing about the headless build:

- `-import` with an absolute Windows path fails with `No file system is defined
  for scheme: C`. Use a path relative to the working directory.
- If the repository's own `.metadata` reports *"doesn't appear to be a CDT
  project"*, point `-data` at an empty scratch directory instead. The project
  imports cleanly there and the repo's workspace metadata is left alone.

### ESP32 (network half)

```sh
pio run -e linkv2   -t upload -t monitor   # bring the link up FIRST
pio run -e tbgateway -t upload -t monitor  # then the gateway
```

`tbgateway` needs no `secrets.h` and no certificates — every credential is in
NVS, written by the tool below. The binary is identical on every unit.

---

## 3. Upload the cloud configuration

```sh
python tools/gw_config_tool.py
```

or, with PlatformIO's interpreter (it already has pyserial):

```sh
%USERPROFILE%\.platformio\penv\Scripts\python.exe tools\gw_config_tool.py
```

Pick the port, **Connect**, fill in the fields, **Write + Save**, then reboot
when it offers.

| Field | NVS key | Notes |
|-------|---------|-------|
| WiFi SSID | `wifi.ssid` | |
| WiFi password | `wifi.pass` | Never read back. Blank = keep what is stored |
| ThingsBoard host | `tb.host` | e.g. `demo.thingsboard.io` |
| Port | `tb.port` | 1883 plain, 8883 TLS |
| Device access token | `tb.token` | From the device's **Credentials** dialog |
| Use TLS | `tb.tls` | See the warning below |
| Device name | `dev.name` | MQTT client id. Defaults to `IoTPLC` |
| Telemetry period | `tb.telemetryMs` | Full snapshot period, 1000–3600000 |

For a production line, the same tool runs headless:

```sh
python tools/gw_config_tool.py --port COM15 \
    --ssid Plant --pass s3cret --host demo.thingsboard.io \
    --token A1_TEST_TOKEN --write --reboot
```

### TLS is not yet verified

Turning on `tb.tls` encrypts the connection but **does not validate the server's
certificate** — `tb_client.cpp` calls `setInsecure()`. Anything able to intercept
the connection can present its own certificate and collect the access token,
which is this device's entire identity. Use it on a trusted network, or add your
server's CA with `setCACert()` first (ThingsBoard Cloud and
`demo.thingsboard.io` use ISRG Root X1). The firmware logs a warning every boot,
and the GUI asks for confirmation, so nobody enables it by accident.

Physical access to the USB port means access to the token, by design. That is
the same trust boundary as the SWD header next to it, so the console does not
pretend otherwise with a password prompt.

---

## 4. Import the dashboard

ThingsBoard → **Dashboards** → **+** → **Import dashboard** →
`dashboards/iiot_gateway_dashboard.json`.

Every widget is bound to the ThingsBoard device named **`IoTPLC`**. That name
has to match the device in your device list exactly - not its label, and not its
access token. For a differently named device, regenerate:

```sh
python tools/make_tb_dashboard.py --device-name plant-a-gw
```

ThingsBoard always routes widget data through an entity alias, so the file still
contains one, but it is a thin wrapper: named `IoTPLC` and filtering on that
name. There is nothing to pick from a dropdown after importing.

The generator exists because a ThingsBoard widget config is large and
version-specific. It downloads each widget type from ThingsBoard's own source,
keeps its published `defaultConfig`, and overrides only the datasource, the RPC
method names and the title — so the file is grounded in real ThingsBoard
defaults rather than invented fields. Regenerate for another release with
`--ref release-3.7.0`.

What you get: four switch controls (relays), one live table of the four inputs,
an I/O history table, and a link-health table.

The inputs are a table rather than four cards on purpose. They are booleans, and
ThingsBoard's value card is a numeric readout — 52 px digits, decimals, units —
which has nothing sensible to show for `true`. The table renders each input with
the cell-style function ThingsBoard uses for booleans in its own demo dashboard:
a filled circle, green when the contact is closed, grey when it is open.

If the dashboard shows nothing, check **Device → Latest telemetry** in
ThingsBoard first. Keys `input1`..`input4` changing there means the gateway is
fine and the problem is in the widget; nothing arriving means the problem is the
device, the token or the device name — and `STATUS` in the config tool will show
`io.inputMask` changing as you close a contact.

---

## 5. The MQTT interface

Telemetry on `v1/devices/me/telemetry`, every `tb.telemetryMs` **and
immediately whenever a relay or input changes**:

| Key | Meaning |
|-----|---------|
| `relay1`..`relay4` | What the pins are actually doing, not what was asked |
| `input1`..`input4` | Debounced inputs |
| `relayMask`, `inputMask` | The same four bits packed, bit 0 = channel 1 |
| `ioQuality`, `ioQualityCode` | `good` / `stale` / `commFail` / … |
| `ioValid`, `ioAgeMs`, `stmStampMs` | Freshness of the snapshot |
| `linkOk`, `linkRttUs`, `linkTimeouts`, `linkCrcErrors`, `linkRetries` | Inter-chip link health |
| `rssi`, `freeHeap`, `uptimeSec` | ESP32 health |

RPC, server-to-device:

| Method | Params | Returns |
|--------|--------|---------|
| `setRelay1`..`setRelay4` | `true`/`false`, `1`/`0`, `"on"`, or `{"value":…}` | The relay's state **after** re-reading the pins |
| `getRelay1`..`getRelay4` | — | Actual state |
| `setRelays` | `0`..`15` bitmask | Resulting mask |
| `getRelays`, `getInputs` | — | Bitmask |
| `getStatus` | — | Object of link and IO diagnostics |

Every `set` confirms by re-reading the pins before replying. A dashboard that
echoes the command back as if it were the state is lying to the operator at
exactly the moment something has come loose.

Quality travels with every value. A reading whose `ioQuality` is not `good` is
not to be trusted as a number, however plausible it looks.

---

## 6. How a dashboard click reaches a contactor

```
dashboard switch
  -> MQTT  v1/devices/me/rpc/request/<id>   {"method":"setRelay2","params":true}
  -> tb_client.cpp            parses, calls LocalIoClient::setRelay(1, true)
  -> stm_link_v2.cpp          WR_CHANNEL, region LOCAL_IO, slot 1, value 1
  -> gw_link.c                handleWriteChannel -> GwLocalIO_WriteSlot
  -> gw_localio.c             stores the commanded bit
  -> GwLocalIO_Step()         drives PD1 (<= GW_LIO_SCAN_MS later)
  -> publishes slots back into the process image
  -> the RPC handler re-reads them and answers with the real state
```

The ESP32 never writes the process image directly. `WR_CHANNEL` asks the driver
that owns the slot, and that driver is the only thing that ever stores to it —
which is what keeps exactly one writer per value when a cloud command and a
scanner both have an opinion. `WR_REGION` remains for the Modbus TCP mirror,
where the network half genuinely owns the data.

---

## 7. Lost-link failsafe

Off by default:

```c
#define GW_LIO_FAILSAFE_MS 0u   /* non-zero: drop every relay after this long */
```

It watches the link's accepted-frame counter, not the time since the last
command, so a relay legitimately left on for an hour never drops while the ESP32
is still talking.

It is off by default on purpose. Whether a lost network should open the contacts
is a property of the machine, not of the firmware — a conveyor should stop, a
heater holding a setpoint probably should not — and a default that silently
opens contactors in the field would be worse than no default. Decide it when you
commission the panel.

---

## 8. When it does not work

| Symptom | Where to look |
|---------|---------------|
| RGB LED yellow | Unprovisioned. Run the config tool |
| RGB LED red | STM32 link down. Run `-e linkv2` and check ECHO |
| RGB LED blue | IO is alive, cloud is not. `STATUS` in the tool, check `cloud.mqttState` |
| RGB LED green | Publishing normally |
| Tool says "No gateway found" | Board is flashed with the wrong environment. Use `-e tbgateway` |
| Tool lists no ports | Bluetooth links are hidden; check the cable, and that the board is not stuck in download mode |
| Switches do nothing, no error | STM32 built without `GW_LOCALIO_ENABLE`. The boot log warns about the missing `GW_CAP_LOCAL_IO` capability |
| `relayN` never changes | Check `ioQuality`. `commFail` means the ESP32 is not reading the image at all |
| Dashboard shows no data, telemetry looks fine | Device name mismatch. Every widget binds to the device named `IoTPLC` — regenerate with `--device-name` |
| MQTT connects then drops | Wrong access token, or a second device connected with the same one |

The console shares the port with the running log, so the log lines that explain
a failure appear in the tool's lower pane. Every protocol reply starts with `+`;
nothing else the firmware prints does. That is how the tool tells answers from
noise without muting the log that diagnoses the problem.
