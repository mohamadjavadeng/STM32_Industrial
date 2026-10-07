# IoT PLC — Status and Roadmap to Prototype 1

Status snapshot of this repository and the plan to finish **prototype 1**: the IoTPLC_CPU board
(STM32H750VBT6 + ESP32-S3-WROOM-1-N16R8, see [`HARDWARE_DESIGN.md`](HARDWARE_DESIGN.md)) built,
running this firmware, and validated on the bench.

> Last update: 2026-10-07. Tick the checkboxes as work lands; update the ladder in §1 when a level
> completes.

---

## 1. Where we are

The design phase is finishing; the build phase has not started.

- **Firmware** is proven on development kits — STM32F407 board + Freenove ESP32-S3.
- **Custom board** schematic is essentially complete (revision 2026-10-03, DRC 0 errors).
  PCB layout has not started.
- **Nothing of prototype 1 exists physically yet**, and no firmware has run on the H750.

```
L0  Concept & architecture            ██████████  done
L1  Dev-kit proof (F407 + Freenove)   ███████░░░  mostly done — Modbus RTU missing in v2
L2  Custom-board schematic            █████████░  ◄── WE ARE HERE (freeze it)
L3  PCB layout                        ░░░░░░░░░░
L4  Fab + assembly                    ░░░░░░░░░░
L5  Bring-up + firmware port to H750  ░░░░░░░░░░
L6  Prototype 1 validated             ░░░░░░░░░░  ◄── GOAL
──────────── after prototype 1 ────────────
v1.1  Ethernet (W5500 on the ESP32)
v2    Embedded-Linux head (STM32MP135) with dual Ethernet
```

### Definition of done for prototype 1

One IoTPLC_CPU board, powered from 24 V:

1. reads a real RS-485 Modbus RTU slave **and** all of its own I/O (16 DI, 12 DO, 4 AI, 2 AO),
2. publishes it to ThingsBoard and accepts commands back,
3. survives a power cut (retentive data intact in FRAM) and a lost network link,
4. can be firmware-updated through the ESP32 (STM32 ROM bootloader path, `HARDWARE_DESIGN.md` §16).

---

## 2. What is done

| Area | Done | Where |
|---|---|---|
| Architecture | Two-chip split (STM32 = control, ESP32 = network), universal process image, cloud-agnostic link contract | [`docs/GATEWAY_ARCHITECTURE.md`](docs/GATEWAY_ARCHITECTURE.md), [`docs/UNIVERSAL_PROCESS_IMAGE.html`](docs/UNIVERSAL_PROCESS_IMAGE.html), [`shared/gw_model.h`](shared/gw_model.h) |
| STM32 link v2 | DMA-driven USART3, process image with per-slot quality and timestamps, opcodes ECHO / SYNC / RD_META / RD_REGION / WR_REGION | `ModbusRTU_AWS/Core/Src/gw_link.c`, `gw_link_port.c`, `gw_image.c` |
| STM32 local I/O | 4 relays (PD0–PD3), 4 inputs (PD4–PD7), debounce, optional lost-link failsafe | `ModbusRTU_AWS/Core/Src/gw_localio.c` |
| ESP32 network half | Link v2 client + bring-up console; ThingsBoard client with telemetry and RPC; NVS provisioning (one binary for every unit, no secrets compiled in) | `IoTHandler`, environments `linkv2` and `tbgateway` |
| PC tooling | Provisioning tool (GUI + headless), ThingsBoard dashboard generator + dashboard JSON | [`tools/`](tools/), [`dashboards/`](dashboards/) |
| End-to-end proof | V2.2 fixed a ThingsBoard switch-widget bug observed on a live dashboard — the cloud path has run for real | commit `8d58abe` |
| Commissioning guide | Wiring, flashing, provisioning, dashboard import, troubleshooting | [`docs/CLOUD_COMMISSIONING.md`](docs/CLOUD_COMMISSIONING.md) |
| Hardware design | 9 schematic pages, two robustness revisions, complete H750 pin map | [`HARDWARE_DESIGN.md`](HARDWARE_DESIGN.md) |
| Forward planning | Expansion bus, Ethernet upgrade, Linux SoC selection | [`docs/EXPANSION_BUS.html`](docs/EXPANSION_BUS.html), [`docs/ETHERNET_UPGRADE.html`](docs/ETHERNET_UPGRADE.html), `IoT PLC — Embedded Linux SoC Selection.docx` |

### Progress against the P1–P8 build order (`UNIVERSAL_PROCESS_IMAGE.html` §09)

| Phase | Content | Status |
|---|---|---|
| P1 | Freeze, measure, `.gitignore` | Partial — no `.gitignore`, no baseline numbers recorded |
| P2 | Shared model header | **Done** |
| P3 | Process image + non-blocking RTU scanner | Image done, **RTU scanner not started** |
| P4 | Link v2 + DMA | **Done** (v1 still present behind `GW_LINK_ENABLE`) |
| P5 | Config in flash (A/B slots) | Not started — not needed for prototype 1 |
| P6 | PC tool v1 (channel editor) | Not started — provisioning tool exists, channel editor does not |
| P7 | Other drivers | Local I/O only |
| P8 | Writes, events, speed | Not started |

---

## 3. Gaps that matter most

1. **Modbus RTU is not live in the v2 firmware.** This is the product's core function.
   `RtuScan_Step()` exists only as a comment in the `main.c` superloop, and the MB_RTU region is
   filled by a demo animation (`GW_IMAGE_DEMO 1`, `ModbusRTU_AWS/Core/Inc/gw_link_cfg.h:257`).
   The cloud currently sees only 4 relays and 4 inputs.

2. **The firmware targets the wrong MCU.** The board uses an STM32H750VBT6 (8 MHz HSE, 128 KB
   internal flash, L1 cache, DTCM). The code targets the STM32F407ZG (25 MHz HSE). The port has
   not started.

3. **H7 DMA trap.** On the H7, 0x20000000 is **DTCM**, which DMA1/DMA2 cannot reach. The check at
   `ModbusRTU_AWS/Core/Src/gw_image.c:150` only rejects the F4's CCM (0x10000000), so on the H7 it
   passes and the link DMA silently transfers nothing. The default CubeIDE H7 linker script places
   `.bss` in DTCM, so this happens by default. D-cache coherency for DMA buffers must also be
   handled (MPU non-cacheable region, or clean/invalidate).

4. **QSPI execute-in-place conflicts with the ESP32 update path.** `HARDWARE_DESIGN.md` §4.1 plans
   to run from QSPI flash; §16 plans firmware updates through the STM32 ROM bootloader. The ROM
   bootloader writes internal flash only, never QSPI.
   **Decision for prototype 1:** keep the application in the 128 KB internal flash (today ≈ 23 KB
   of code at `-O0`) and use QSPI for data only. If the application outgrows 128 KB, the
   STM32H743VIT6 is the same LQFP100 package with 2 MB flash — verify pin compatibility against
   the datasheet before relying on it.

5. **The ESP32 firmware assumes the Freenove board.**
   - Status LED: the code drives a WS2812 on GPIO48 (`IoTHandler/src/tb_main.cpp:163`); the new
     board has a plain red LED on GPIO2.
   - Side-band lines: `PIN_STM_EVENT` / `PIN_STM_RESET` are still `-1` in
     `IoTHandler/include/config.h`, but the board now wires IO4 (`STM_EVENT`), IO5
     (`STM_NRST_CTL`) and IO6 (`STM_BOOT0_CTL`).

6. **Schematic items still open.**
   - ESP32 BOOT button on IO0 (`HARDWARE_DESIGN.md` §15 item 8 — only the USB ESD part was added).
   - F501 is a 1 A fast fuse in front of 220 µF hold-up plus bulk capacitance — consider slow-blow
     or inrush limiting (§16 open point).
   - EasyEDA *Device Standardization* warnings (21 remaining).
   - Ethernet decision (see §5).

7. **TLS does not verify the server** (`setInsecure()` in `tb_client.cpp`). Acceptable on the
   bench, not at a pilot site.

8. **Repository housekeeping.**
   - No `.gitignore`; `Debug/`, `.metadata/` and `tools/__pycache__/` are tracked.
   - `HARDWARE_DESIGN.md`, the SoC selection `.docx` and `docs/ETHERNET_UPGRADE.html` are untracked.
   - `CLAUDE.md` still describes only the v1 link.
   - The EasyEDA source files are not in the repository.

---

## 4. Roadmap — two parallel tracks

Firmware work does not need to wait for the PCB. Track B runs on the existing F407 + Freenove
bench rig while track A is in layout and fab. The tracks merge at board bring-up.

```
NOW                                                                    PROTOTYPE 1
 │                                                                          ▲
 ├─ HW ─[A1 Freeze schematic]─[A2 Layout]─[A3 DFM + order]─[A4 Fab]──┐      │
 │                                                                   ├─[C1 Bring-up]─[C2 H750 port]─[C3 Drivers]─[C4 Validate]
 └─ FW ─[B1 Housekeeping]─[B2 RTU scanner]─[B3 I/O model]─[B4 ESP32]─┘
        (on the F407 + Freenove bench rig, in parallel)
```

### Track A — Hardware

#### A1. Freeze the schematic
- [ ] Make the decisions in §5 (DO type, Ethernet, control logic, expansion bus).
- [ ] Add the ESP32 BOOT button on IO0.
- [ ] Resolve the F501 fuse vs. hold-up inrush question.
- [ ] Confirm in AN2606 that the H7 ROM bootloader supports USART3 on PB10/PB11 — the whole
      §16 update path depends on it.
- [ ] Run Device Standardization; reduce DRC warnings to intentional ones only.
- [ ] Export schematic PDF + BOM into `hardware/` in this repository.
- [ ] Tag the frozen schematic revision.

**Done when:** DRC shows only intentional warnings and the revision is tagged.

#### A2. PCB layout
- [ ] Isolation gaps under DI/DO optocouplers and the ADM2587E.
- [ ] Analog zoning (AI dividers, burden resistors, op-amps away from DO and bucks); single-point VSSA return.
- [ ] Tight buck converter loops (LM5160, TPS563201), thermal vias.
- [ ] QSPI length matching; USB D+/D− routed through the USBLC6 pads.
- [ ] 4-layer stack-up.

**Done when:** DRC/DFM clean.

#### A3. DFM and order
- [ ] Check stock: LM5160, ADM2587E, XTR111, TLP290-4, TLP291-4, FM25V20A, DS3231SN, W25Q256JVEIQ.
- [ ] Order 5 boards + stencil.

#### A4. Fabrication and assembly

### Track B — Firmware on the bench rig

#### B1. Housekeeping
- [ ] Add `.gitignore` for `ModbusRTU_AWS/Debug/`, `.metadata/`, `__pycache__/`.
- [ ] Commit `HARDWARE_DESIGN.md`, the SoC selection `.docx`, `docs/ETHERNET_UPGRADE.html`, this file.
- [ ] Tag the dev-kit baseline (e.g. `v2.2-devkit`).
- [ ] Update `CLAUDE.md` to describe link v2, local I/O and the `tbgateway` environment.

**Done when:** `git status` is clean after a build.

#### B2. Non-blocking Modbus RTU scanner (P3)
- [ ] Convert `modbusMaster.c` to a state machine (no blocking `HAL_UART_Receive`).
- [ ] Add `RtuScan_Step()` to the superloop, filling the MB_RTU region.
- [ ] Set `GW_IMAGE_DEMO 0`.
- [ ] Handle slave exception replies (`MODBUS_EXCEPTION_MASK` 0x80).
- [ ] Test against the real slave (ID 2, holding 4097, coils 1281).

**Done when:** unplugging the slave changes only its own tags to `COMM_FAIL`, and the link round
trip stays under 5 ms.

#### B3. Full I/O model
- [ ] Size the LOCAL_IO region for 16 DI, 12 DO, 4 AI, 2 AO plus diagnostics (24 V level, board
      temperature, watchdog resets).
- [ ] Move pin tables into per-board files (e.g. `board_f407.h` / `board_h750.h`) so
      `gw_localio.c` is not rewritten for the port.

**Done when:** the ESP32 side and the dashboard can be built against the new model before the
board arrives.

#### B4. ESP32 board profile and cloud
- [ ] Publish MB_RTU and the full LOCAL_IO set to ThingsBoard.
- [ ] Generalize `localio_client` from 4 relays to N channels.
- [ ] Add a custom-board pin profile: LED on GPIO2, `STM_EVENT` = IO4, `STM_NRST_CTL` = IO5,
      `STM_BOOT0_CTL` = IO6.
- [ ] Regenerate the dashboard with `tools/make_tb_dashboard.py`.
- [ ] Replace `setInsecure()` with `setCACert()` (ISRG Root X1 for ThingsBoard Cloud).

**Done when:** the dashboard shows real RS-485 data.

### Track C — Merge on the real board

#### C1. Bring-up
- [ ] Power from a current-limited supply with no firmware; check `+24V_PROT`, `+5V`, `VCC`,
      `VISO_485`, `VREF+`.
- [ ] **Remove jumper J901** (or kick the watchdog on PE5 from the very first boot) — otherwise
      the TPS3828 resets the board every 1.6 s while you debug.
- [ ] Connect over SWD (J903); flash a blink.
- [ ] Flash the ESP32 over USB-C.

**Done when:** all rails are correct and both chips can be programmed.

#### C2. Port to the H750
- [ ] New CubeMX project for the H750 (keep `ModbusRTU_AWS` as the F407 dev-kit reference).
- [ ] Clock tree from the 8 MHz HSE.
- [ ] Place the process image and UART DMA buffers in AXI SRAM (0x24000000) or SRAM1
      (0x30000000) via a linker section; update the address check in `gw_image.c:150` to
      reject DTCM.
- [ ] MPU configuration and cache handling for DMA buffers.
- [ ] Application in internal flash; QSPI for data only.

**Done when:** ECHO round-trips over link v2 between the H750 and the ESP32 on the same board.

#### C3. Drivers
- [ ] DI (PD0–PD7, PE7–PE14, active low) and DO (12 channels, pin list in `HARDWARE_DESIGN.md` §8).
- [ ] ADC, 8 channels (V and I per AI), with wire-break and over-range detection.
- [ ] DAC + XTR111 (`AO2_OD` on PE15, `AO2_EF` on PB7).
- [ ] Status LEDs (RUN PE6, ERR PC13, COM PC12) and watchdog kick (PE5) from the main loop.
- [ ] Power-fail detection (PC3_C) → save retentive data to FRAM (FM25V20A on SPI2).
- [ ] DS3231 RTC and the I²C LCD (I2C1, PB8/PB9).
- [ ] RS-232 on USART2.
- [ ] ESP32 updates the STM32 through BOOT0/NRST and the ROM bootloader.

**Done when:** every I/O channel appears in ThingsBoard.

#### C4. Validation
- [ ] Every DI, DO, AI and AO channel tested at its terminals.
- [ ] 24 V miswire on each AI input (V and I terminals) — board survives.
- [ ] 24-hour RS-485 soak with the real slave.
- [ ] Pull 24 V 100 times — retentive data in FRAM intact every time.
- [ ] Watchdog recovery (hang the main loop deliberately).
- [ ] Lost-link failsafe and cloud reconnect after Wi-Fi loss.
- [ ] Thermal check at full DO load (ULN2803 packages, bucks).
- [ ] Firmware update of the STM32 through the ESP32.

**Done when:** the checklist passes — prototype 1 is complete.

---

## 5. Decisions needed before layout

Changing any of these after layout means another board spin.

| # | Decision | Recommendation |
|---|---|---|
| 1 | **DO type:** sinking ULN2803 (as drawn) or sourcing high-side switches? | Keep ULN2803 for prototype 1, unless the first customer needs PNP outputs. `HARDWARE_DESIGN.md` §18 already lists high-side switches as a priority-1 industrial upgrade for a later revision. |
| 2 | **Ethernet on prototype 1?** | Break out ESP32 IO9–IO14 + 3V3 + GND to a header. Zero risk, no cost, and a W5500 module can be plugged in to develop v1.1 firmware. |
| 3 | **Does control logic run on the STM32, or is it purely a gateway?** | Yes — the product is a PLC. Decide before B2, because it fixes the write-ownership rule for the process image. |
| 4 | **Expansion bus** | Keep the hardware on the board; defer its firmware until after prototype 1, since no modules exist yet. |

---

## 6. Explicitly out of scope for prototype 1

- AWS IoT and generic MQTT back ends — prototype 1 is ThingsBoard only. (The legacy
  `freenove_esp32_s3_wroom` AWS environment still uses link v1; port or retire it later.)
- Configuration in flash (P5) and the channel-editor PC tool (P6).
- CAN, Modbus TCP, expansion modules, QSPI execute-in-place, secure element.
- W5500 Ethernet (v1.1), STM32MP135 Linux head (v2).
- Type tests and certification (EN 61131-2, IEC 61000-6-2/-4).
