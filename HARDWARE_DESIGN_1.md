# IoT PLC — Hardware Design Notes (IoTPLC_CPU)

EasyEDA Pro project **IoTPLC_CPU** — head board of the IIoT PLC described in this repository
(`ModbusRTU_AWS` = industrial half on STM32, `IoTHandler` = network half on ESP32-S3).

This file records *what* is on each schematic page, *why* it was designed that way, the complete
MCU pin map, and what is still open. Keep it in sync with the schematic.

> Status: schematic capture, pre-layout. Last update: 2026-09-30.

---

## 1. System overview

| Block | Main parts | Page |
|---|---|---|
| Power 24 V → 5 V → 3.3 V | LM5160 buck, TPS563201 buck, fuse / reverse / TVS | 2. 5VDC |
| MCU (industrial half) | STM32H750VBT6 (LQFP100), 8 MHz HSE | 1. CPU_part |
| External memory | W25Q256 (QSPI, 32 MB), W25Q64 (SPI, 8 MB, latch data) | 3. Flash_memory |
| Network half | ESP32-S3-WROOM-1-N16R8, USB-C, I²C LCD port | 4. Connection |
| Digital / analog inputs | 16 × 24 V DI (opto), 4 × AI (0–10 V + 4–20 mA) | 5. DI & AI |
| Digital / analog outputs | 12 × isolated transistor DO, AO 0–10 V, AO 4–20 mA | 6. Do & AO |
| Field communication | Isolated RS-485 (Modbus RTU), RS-232 (DB9) | 7. communication |
| Supervision | External watchdog, status LEDs, protective earth | 8. Watchdog & protection |
| Expansion | SPI3 backplane head, 8 slots | 9. Expansion bus |

Global nets: `+24V` (raw terminal), `+24V_PROT` (after fuse + reverse diode), `+5V`, `VCC` (= 3.3 V),
`GND`. Isolated / field domains: `GND_ISO` + `VISO_485` (RS-485), `+24V_DO` / `0V_DO` (DO field),
`COMA` / `COMB` (DI field commons), `+24V_AO` (AO supply), `PE` (protective earth).

Designator ranges per page: 5VDC 5xx, Connection 3xx, Flash 4xx, DI&AI 6xx, DO&AO 7xx,
communication 8xx, watchdog 9xx, expansion 10xx (CPU page keeps U1/Cx/Rx, plus C101/C102/R101).

---

## 2. Power supply (page 5VDC)

### 2.1 Input protection
`CN2` (3.81 mm) → **F501** 1 A fast fuse (0466001.NR, 63 V) → **D502 SS36** series Schottky
(reverse-polarity: blocks, fuse does not blow; ~0.45 V drop) → **D501 SMBJ33A** TVS → bulk
47 µF/50 V + 2 × 10 µF/50 V. Order is deliberate: the TVS sits *after* the diode so a reversed
supply cannot drive current through it and blow the fuse.
Protected node is named **`+24V_PROT`** (feeds the expansion backplane).

### 2.2 24 V → 5 V: LM5160 (U501)
* Spec: 18–32 V in, 5 V / 1.5 A out, COT control, fsw ≈ 500 kHz.
* RON = 100 k (sets fsw; on-time at 32 V ≈ 310 ns, well above t_on(min)).
* FB: 30 k / 20 k → 2.0 V × (1 + 30/20) = 5.00 V.
* L501 15 µH (FXL0630-150-M, Isat ≥ 3 A), ripple ≈ 0.56 App at 32 V.
* Ripple injection network (Rr/Cr/Cac) is required because the output caps are ceramic (COT needs
  ≥ ~25 mV ripple at FB).
* EN/UVLO divider 100 k / 10 k → start ≈ 14–16 V; FPWM → GND (DCM at light load).
* 2 × 22 µF/25 V output; BST 100 nF; VCC 1 µF; SS 22 nF.
* Layout: short VIN–PGND input loop, small SW copper, FB away from SW/L, AGND–PGND single point
  under the exposed pad, thermal vias.

### 2.3 5 V → 3.3 V: TPS563201 (U506) — replaces the AMS1117
The LDO would dissipate ≈ (5 − 3.3) × 0.7 A ≈ 1.2 W in SOT-223. The synchronous buck
(≈ 90 %) dissipates ≈ 0.2 W.
* VOUT = 0.768 V × (1 + 33.2 k / 10 k) = 3.32 V (R507 / R508, 1 %).
* L502 2.2 µH (FXL0630-2R2-M), 2 × 22 µF out, 10 µF + 100 nF in, 100 nF VBST, EN tied to VIN.

### 2.4 Power budget (3.3 V)
ESP32-S3 Wi-Fi TX peaks ≈ 500 mA, H750 ≈ 150–250 mA, flashes / LEDs / logic ≈ 100 mA → design for
≥ 0.8 A on VCC. 5 V rail (1.5 A) also feeds the LCD, relay-driver-free logic and the expansion
+5 V (limited to 500 mA by PTC).

---

## 3. MCU core (page CPU_part)

* **STM32H750VBT6**, LQFP100. Note: firmware in `ModbusRTU_AWS` targets **STM32F407ZG** → must be
  ported (clock tree, HAL, linker, pin map below).
* HSE 8 MHz crystal (X1) with 22 pF.
* NRST: reset button SW1 + 100 nF; also driven by the external watchdog (open-drain).
* **VCAP pins 48 / 73: C101 / C102 = 2.2 µF X5R each** — mandatory for the internal regulator.
  Place directly at the pins with short GND vias.
* **BOOT0: R101 10 k pull-down** (always boot from internal flash).
* **VBAT tied to VCC** (no RTC battery yet — change to coin cell / supercap through a diode if the
  32.768 kHz RTC is added on PC14/PC15).
* Decoupling 100 nF per VDD pin + 1 µF bulk.
* ⚠ `VREF+` / `VDDA` currently share the `VCC` net (DRC warns "Vref+ / VCC multiple net names").
  Recommended: ferrite bead + 1 µF + 100 nF, optionally a REF3030 3.0 V reference for the AI.
* ⚠ SWD header not yet placed (PA13 / PA14 / NRST / 3.3 V / GND, optionally PB3 SWO — but PB3 is
  now SPI3_SCK).

---

## 4. External memory (page Flash_memory)

### 4.1 W25Q256JVEIQ — QSPI, memory-mapped (U4)
| Flash | Net | MCU | AF |
|---|---|---|---|
| CLK | QSPI_CLK via R402 22 Ω (place near MCU) | PB2 | AF9 |
| CS# | QSPI_NCS + R401 10 k pull-up | PB6 | AF10 |
| IO0 | QSPI_IO0 | PD11 | AF9 |
| IO1 | QSPI_IO1 | PD12 | AF9 |
| IO2 | QSPI_IO2 | PE2 | AF9 |
| IO3 | QSPI_IO3 | PD13 | AF9 |

* "IQ" part: QE bit set from factory → IO2/IO3 usable immediately.
* Memory-mapped at 0x9000_0000–0x91FF_FFFF (`FlashSize = 24`).
* Use **4-byte-address commands** (0xEC, 6 dummy cycles) so a reset during 4-byte mode is harmless.
* MPU region for 0x90000000; bootloader in internal 128 KB jumps to QSPI; STM32CubeProgrammer
  needs an external loader (.stldr).
* Clock ≤ 100–120 MHz (part rated 133 MHz).
* Alternative considered: Bank 2 (PB2 + PE7–PE10 + PC11) routes better (5 adjacent pins), but was
  not chosen; PE7–PE14 are now DI9–16.

### 4.2 W25Q64JVSSIQ — SPI2, latch / retentive data (U5)
PB13 SCK, PB14 MISO, PB15 MOSI, **PB12 LATCH_CS** (R403 10 k pull-up); /WP, /HOLD tied to VCC.
* Flash endurance ≈ 100 k erase cycles / 4 KB sector, erase 45–400 ms → use wear-levelling
  (log / LittleFS), double copy + CRC, and write on power-fail (see §12) rather than on every change.
* **Drop-in alternative:** SPI FRAM (MB85RS64V / FM25V02A) has the same SOIC-8 pinout, ~10¹⁴ writes,
  no erase — preferred for PLC retentive memory.

---

## 5. Network half & HMI (page Connection)

### 5.1 ESP32-S3-WROOM-1-N16R8 (U2)
* 3V3 → `VCC`, 10 µF + 100 nF at the module.
* EN: R301 10 k to VCC + C303 1 µF (Espressif RC) + **SW2 reset button**.
* **Inter-chip link (USART1 of ESP32 ↔ USART3 of STM32):**
  ESP32 GPIO17 (TX) → `STM_RX` → STM32 PB11; STM32 PB10 → `STM_TX` → ESP32 GPIO18 (RX).
  115200 8N1 (per `config.h` / `main.c`). Same board, same GND → no isolator needed.
* **Wi-Fi status LED:** GPIO2 → R304 1 k → red LED (active high).
* **USB-C (J301, TYPE-C-31-M-12):** D+ (A6/B6) → GPIO20, D− (A7/B7) → GPIO19; CC1/CC2 5.1 k to GND
  (sink); shell → GND; VBUS pins joined as `USB_VBUS` but not connected to +5V (no back-feed).
* Avoided ESP32 pins: 0/3/45/46 (strapping), 19/20 (USB), 26–32 (flash), 35–37 (octal PSRAM),
  43/44 (UART0 debug console).

### 5.2 Character LCD (I²C backpack, J302 1×4 2.54 mm: GND, +5V, SDA, SCL)
* STM32 **I2C1: PB8 SCL, PB9 SDA** (PB6/PB7 not usable: PB6 = QSPI_NCS).
* **BSS138 level shifters** (Q301/Q302) with 4.7 k pull-ups on both sides — PCF8574 at 5 V needs
  V_IH ≥ 3.5 V.
* **SRV05-4** ESD array on SDA/SCL (front-panel cable).
* Address 0x27 (PCF8574) / 0x3F (PCF8574A), 100 kHz, cable ≤ ~30 cm.

---

## 6. Digital inputs — 16 × 24 VDC (page DI & AI)

* IEC 61131-2 Type 1/3 behaviour: R_in 4.7 k (1206) + 1 k shunt across the opto LED → ON ≥ ~11 V,
  OFF ≤ 5 V, max 30 V.
* **TLP290-4 (AC input)** optocouplers, 3750 Vrms → each group works with **PNP (sourcing) or NPN
  (sinking)** sensors: COM to 0 V for PNP, COM to +24 V for NPN.
* Groups: J601 = DI1–8 + **COMA**, J602 = DI9–16 + **COMB**.
* MCU side: 47 k pull-up + green LED (2.2 k) per channel — LED shows the real input state in hardware;
  signal is **active low** at the MCU.
* Field side (DIx_F, COMx) must never touch GND — keep isolation gap under the optos.
* **MCU map:** DI1–8 = **PD0–PD7**, DI9–16 = **PE7–PE14** (one port read per group).

---

## 7. Analog inputs — 4 channels, V and I (page DI & AI)

Each channel has **separate V and I terminals** (J603: V, I, COM per channel) and **two ADC pins**.

| Path | Protection | Scaling | Buffer |
|---|---|---|---|
| 0–10 V | SMAJ30CA at terminal, 23.2 k series, BAT54S clamp | 23.2 k / 10 k 0.1 % → 3.01 V FS, Zin 33 kΩ | TLV9064 RRIO follower, 100 Ω / 10 nF ADC driver |
| 4–20 mA | SMAJ30CA, **50 mA PTC + SMAJ6.5CA** (forces PTC trip on 24 V miswire), BAT54S clamp | 150 Ω 0.1 % burden → 0.6–3.0 V | same |

* Both paths survive a 24 V miswire. Headroom above 3.0 V allows wire-break (< 3.6 mA) and over-range
  (> 21 mA) detection.
* **MCU map (ADC1/ADC2 capable, dual simultaneous possible):**
  AI1 V/I = **PC0 / PA0**, AI2 = **PA6 / PA7**, AI3 = **PC4 / PC5**, AI4 = **PB0 / PB1**.
  (AI1_I moved from PC1 to PA0 because of a routing clash.)
* Not isolated (shares GND). Upgrade path: isolated DC-DC + AMC1311 / ISO224 per channel.
* Layout: dividers / burden resistors / op-amps away from DO, relays and buck converters; analog
  return to VSSA single point.

---

## 8. Digital outputs — 12 × isolated transistor (page Do & AO)

Chain: **GPIO → 470 Ω → TLP291-4 → 4.7 k → ULN2803A input (47 k pull-down) → DOx_OUT**

* Galvanic isolation by 3 × TLP291-4; field side powered only from **`+24V_DO` / `0V_DO`** (J702).
* **Sinking (NPN) outputs**: load between +24V_DO and DOx. ULN COM = +24V_DO → built-in flyback
  diodes for inductive loads.
* Green LED per channel (10 k from +24V_DO) shows the real output state.
* Field supply protection: SS36 reverse diode + SMBJ33A + 10 µF + 100 nF. Fuse the field 24 V
  externally.
* Rating: ≈ 300 mA / channel, ≤ ~1.2 W per ULN2803 package (Vce(sat) ≈ 1 V).
* Outputs are OFF during reset (GPIO hi-Z → opto LED dark; ULN inputs pulled down).
* **Relay-ready:** keep everything up to `DOx_OUT` and connect 24 V relay coils instead of terminals
  in the next revision — no MCU / firmware change.
* Terminals: J701 = DO1–8, J702 = DO9–12 + 24V_DOIN + 0V_DO. Unused ULN channels: inputs to 0V_DO.
* **MCU map:** DO1–12 = **PD8, PD9, PD10, PD14, PD15, PC6, PC7, PC8, PC9, PA8, PA15, PC10**.

---

## 9. Analog outputs (page Do & AO)

* Supply: `+24V` → 50 mA PTC → SS36 → **`+24V_AO`** (10 µF + 100 nF).
* **AO1 0–10 V:** DAC1 **PA4** → OPA2171 non-inverting, gain 3.32 (23.2 k / 10 k 0.1 %) → 47 Ω →
  SMAJ15CA → terminal. Op-amp current-limits a short. Second OPA2171 section parked as grounded
  follower.
* **AO2 4–20 mA:** DAC2 **PA5** → **XTR111**, I_out = 10 × V_IN / 1.5 k (0.6–3.0 V → 4–20 mA),
  BSS84 pass FET, SS36 back-feed block, SMAJ30CA TVS; load up to ~750–900 Ω.
  * `AO2_OD` (**PE15**, moved from PB5): high = output disabled; 10 k pull-up keeps AO2 off during
    reset.
  * `AO2_EF` (**PB7**): open-drain, low = open loop / wire break.
* Terminal J703: AO1_V, GND, AO2_I, GND. AO is not isolated.

---

## 10. Field communication (page communication)

### 10.1 RS-485 / Modbus RTU — isolated
* **ADM2587E**: 2.5 kVrms isolated transceiver with integrated isolated DC-DC (`VISO_485`,
  `GND_ISO`).
* STM32 **USART1: PA9 TX, PA10 RX, PA11 DE** (= `RS485_EN` in firmware); DE and /RE tied together.
* SM712 TVS (−7 / +12 V) on A/B; **J802 jumper = 120 Ω termination** (only at the two bus ends);
  true fail-safe receiver → no bias resistors.
* Terminal J801: A, B, GND_ISO (shield / reference).
* 10 µF + 100 nF on both sides of the barrier.

### 10.2 RS-232 — DB9
* **MAX3232E** (±15 kV ESD), 5 × 100 nF charge-pump caps.
* STM32 **USART2: PA2 TX, PA3 RX** (UART8 on PE0/PE1 was rejected: routing clash; PE0/PE1 are now
  expansion-bus address lines).
* DB9 male, DTE (PC-style): pin 2 RXD in, pin 3 TXD out, pin 5 GND, shell GND. Use a null-modem
  cable to other DTE devices. Second MAX3232 channel unused (T2IN → GND) — available for RTS/CTS.

---

## 11. Watchdog, status LEDs, protective earth (page Watchdog & protection)

* **TPS3828-33** supervisor + watchdog: reset below 2.93 V, **1.6 s watchdog**, **open-drain** RESET
  → `NRST` (coexists with the reset button and internal MCU resets). MR# tied high.
  * **WDI ← PE5** through jumper **J901** — remove the jumper for breakpoint debugging (WDI floating
    disables the watchdog; brown-out reset stays active).
  * Firmware: toggle WDI from the main loop / idle task, not from a timer ISR.
* **Status LEDs** (1 k each, ~1.3 mA — PC13 can only source ~3 mA):
  PWR (green, VCC), RUN (green, **PE6**), ERR (red, **PC13**), COM (yellow, **PC12**).
* **Protective earth:** J902 (pin 1 = PE to DIN rail / cabinet earth, pin 2 = PE for cable
  shields). Logic GND ↔ PE via **1 MΩ (1206) ∥ 1 nF / 2 kV (1808)** — bleeds static, gives EMC a
  path, avoids a hard ground loop.

---

## 12. Expansion bus — head side (page Expansion bus)

Implements `docs/EXPANSION_BUS.html`: master-clocked **SPI backplane, CS per slot, 8 slots**.

* **SPI3 master:** PB3 SCK, PB4 MISO, PB5 MOSI (≈ 10.5 MHz). 33 Ω series on SCK / MOSI / SYNC at the
  head only; 10 k pull-down on MISO (empty slot reads defined level, CRC fails → `GW_Q_COMM_FAIL`).
* **Slot select: 74LVC138** — A0–A2 = **PE0 / PE1 / PE3**, /EN = **PE4** with 10 k pull-up
  (all CS high during reset). Sequence: /EN high → set address → /EN low → transfer.
  (Saves pins vs. the PE0–PE7 CS lines proposed in the doc — those pins are now DI / LEDs.)
* **ATTN** (**PC11**, EXTI falling): open-drain from modules, 4.7 k pull-up at the head.
* **SYNC** (**PA1**): optional broadcast latch edge.
* **Connector J1001 2 × 10 box header:**

| Pin | Signal | Pin | Signal |
|---|---|---|---|
| 1 | +24V_EXP | 2 | +24V_EXP |
| 3 | +5V_EXP | 4 | GND |
| 5 | EXP_SCK | 6 | GND |
| 7 | EXP_MOSI | 8 | EXP_MISO |
| 9 | GND | 10 | EXP_ATTN |
| 11–18 | EXP_CS0 … EXP_CS7 (odd/even alternating) | | |
| 19 | EXP_SYNC | 20 | GND |

* Power: `+24V_PROT` → 1 A fuse → `+24V_EXP`; `+5V` → 500 mA PTC → `+5V_EXP`; 10 µF each.
* Slot address ADDR0–3 is strapped **on the backplane** per slot; the backplane routes CSn only to
  slot n. SPI is for a backplane only — use CAN if modules move to another enclosure.
* PB3 = SWO → SWO trace is lost (SWD still works).

---

## 13. Complete STM32H750VBT6 pin map

| Function | Pins |
|---|---|
| HSE | PH0 / PH1 (8 MHz) |
| QSPI W25Q256 | PB2 CLK, PB6 NCS, PD11 IO0, PD12 IO1, PE2 IO2, PD13 IO3 |
| SPI2 W25Q64 | PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS |
| USART3 ↔ ESP32 | PB10 TX, PB11 RX |
| USART1 RS-485 | PA9 TX, PA10 RX, PA11 DE |
| USART2 RS-232 | PA2 TX, PA3 RX |
| I2C1 LCD | PB8 SCL, PB9 SDA |
| DI1–16 | PD0–PD7, PE7–PE14 |
| AI (V/I) | PC0/PA0, PA6/PA7, PC4/PC5, PB0/PB1 |
| DO1–12 | PD8, PD9, PD10, PD14, PD15, PC6, PC7, PC8, PC9, PA8, PA15, PC10 |
| AO | PA4 (DAC1), PA5 (DAC2), PE15 AO2_OD, PB7 AO2_EF |
| Watchdog / LEDs | PE5 WDI, PE6 RUN, PC13 ERR, PC12 COM |
| Expansion bus | PB3/PB4/PB5 SPI3, PE0/PE1/PE3 A0–A2, PE4 /EN, PC11 ATTN, PA1 SYNC |
| Debug | PA13 SWDIO, PA14 SWCLK (header not yet placed) |
| **Free** | **PC1, PC2_C, PC3_C, PA12**, PC14/PC15 (reserve for LSE / RTC) |

### Ethernet
Native RMII is **not** possible without large rework (needs PA1, PA2, PC1, PA7, PC4, PC5, PB11,
PB12, PB13 — 7 already used). Recommended: **W5500 on SPI2** (shared with the latch flash):
CS / INT / RST on PA12 / PC1 / PC2_C + RJ45 with magnetics + 25 MHz crystal.

---

## 14. Firmware impact (port F407 → H750)

* New MCU: clock tree, HAL F4 → H7, linker (128 KB internal + QSPI XIP), MPU / cache setup.
* `gw_localio.c` pin tables must change: **relays were PD0–PD3 and inputs PD4–PD7** — now DI1–8 use
  PD0–PD7 and DO use the list in §13. PD12–PD15 are no longer LEDs (PD12/PD13 = QSPI).
* DI are active low (`GW_LIO_DI_ACTIVE_LOW 1` still valid); DO active high through the opto.
* New drivers: ADC (8 channels, dual mode), DAC (2), XTR111 OD/EF, I2C LCD, USART2 RS-232,
  SPI3 expansion master + 74LVC138 select, watchdog kick, status LEDs.
* ESP32 `config.h`: `PIN_STM_EVENT` / `PIN_STM_RESET` are still −1 (lines not wired yet).

---

## 15. Open items / next steps

1. **SWD header** (PA13/PA14/NRST/3.3 V/GND) — required for programming.
2. **VREF+/VDDA** filtering (ferrite + 1 µF + 100 nF), fix the `Vref+`/`VCC` dual-name net.
3. **Side-band GPIOs** from `docs/GATEWAY_ARCHITECTURE.md`: `STM_EVENT` (STM32 → ESP32 GPIO4) and
   `ESP_EN` / `ESP_BOOT` (STM32 → ESP32 EN / IO0) — cheap now, impossible to retrofit.
4. **RTC:** 32.768 kHz crystal on PC14/PC15, VBAT backup (coin cell / supercap + diode).
5. **Power-fail detect:** `+24V_PROT` divider → ADC / comparator + hold-up on 5 V, to flush latch
   data to W25Q64 / FRAM before collapse.
6. **Ethernet:** W5500 on SPI2 (see §13).
7. **CAN** (FDCAN1) if remote expansion is needed — pins would have to be freed.
8. **USB ESD** (USBLC6-2SC6) on the ESP32 USB-C and an ESP32 **BOOT button** on IO0.
9. **Common-mode choke** on the 24 V input (IEC 61000-4-4/-5); EMC review.
10. **Test points:** +24V_PROT, +5V, VCC, VISO_485, +24V_DO, +24V_AO, GND.
11. **PCB layout:** isolation gaps (DI/DO optos, ADM2587E), analog zoning, buck converter loops,
    QSPI length matching, thermal vias (LM5160, TPS563201, ULN2803).
12. **Mechanical:** DIN-rail enclosure, pluggable 3.81 / 5.08 mm terminals, conformal coating,
    −40 … +85 °C rated parts.
13. Run *Device Standardization* in EasyEDA (values were typed manually → "attributes don't match
    supplier part" warnings).

---

## 16. Revision — robustness additions (2026-09-30)

| # | Page | What was added | Designators |
|---|------|----------------|-------------|
| 2 | CPU_part | VDDA/VREF+ filter: ferrite bead from VCC to net `Vref+` (existing C5/C8 100 nF + C7/C9 1 µF stay as the VDDA/VREF+ decoupling) | FB1 (BLM18PG221SN1D) |
| 4 | Connection | USB ESD on ESP32 USB-C: USBLC6-2SC6, I/O1 = pins 1/6 (USB_DP), I/O2 = pins 3/4 (USB_DM), VBUS pin 5, GND pin 2. **PCB: route D+/D− through the chip pads (connector → 1/3, 6/4 → ESP32).** | U302 |
| 5 | Connection / CPU / WDT | Side-band lines (see below) | Q303, Q304, R309–R312 |
| 6 | Watchdog & protection | Power-fail detect: `+24V_PROT` → R906 100k / R907 10k → `POWER_FAIL` (PC3_C), C903 100 nF, D904 BAT54S clamp to VCC/GND. Hold-up C904 220 µF/50 V on `+24V_PROT` | R906, R907, C903, D904, C904 |
| 8 | 5VDC | 24 V input EMI filter between CN2 and F501: common-mode choke ACM7060-701 (4 A) + C520 100 nF X7R 50 V across the line | L503, C520 |
| 9 | communication | DB9 shell (MH1/MH2) → `PE` (was GND). `GND_ISO` → `PE` through R808 1 M ∥ C810 1 nF/2 kV | R808, C810 |
| RTC | Flash_memory | DS3231SN# (SOIC-16, TCXO ±3.5 ppm, −40…+85 °C) on I2C1 (0x68, shared with LCD; pull-ups R305/R307). VCC decoupled by C410. VBAT from soldered CR2032 (pin-type cell, no holder) `BT401` with C411. N.C. pins 5–12 tied to GND per datasheet. INT/SQW, 32kHz, RST# left open | U410, BT401, C410, C411 |

### Side-band lines & "program STM32 over USB"

| Net | From → To | Purpose |
|-----|-----------|---------|
| `STM_EVENT` | STM32 PC1 → ESP32 IO4 | STM32 → ESP32 "data ready / alarm" interrupt |
| `ESP_EN_CTL` | STM32 PA12 → Q304 (2N7002, 100k gate pull-down) → `ESP_EN` | STM32 can hard-reset a hung ESP32 (open-drain) |
| `STM_NRST_CTL` | ESP32 IO5 → Q303 (2N7002) → `WDT_MR` (TPS3828 MR#, R312 10k pull-up) | ESP32 resets STM32 via supervisor MR#, so the push-pull RESET# output never fights a second driver |
| `STM_BOOT0_CTL` | ESP32 IO6 → R311 1k → `BOOT0` (R101 10k pull-down stays) | ESP32 selects STM32 system bootloader |

Firmware flow for USB update: PC/web → ESP32 USB-C (native USB, IO19/IO20) → ESP32 sets IO6 high, pulses IO5 → STM32 boots into ROM bootloader → ESP32 streams the image over USART3 (PB10/PB11, AN3155 protocol) → IO6 low, pulse IO5 → run. Same path enables OTA over Wi-Fi.
Notes: the ESP32 ROM bootloader uses UART0 (IO43/44), not the STM32 link, so an STM32-driven ESP_BOOT line would not help — ESP32 is flashed through its own USB-C. STM32 native USB DFU is not possible because PA11 is RS485_DE.

### Updated free MCU pins
Used now: PC1 (STM_EVENT), PC3_C (POWER_FAIL), PA12 (ESP_EN_CTL). Still free: PC2_C (analog only), PC14/PC15, PA13/PA14 (reserve for SWD).

### Open points from this revision
- F501 1 A fast fuse + bulk/hold-up capacitance → consider a slow-blow (T) fuse or inrush limiting.
- DRC after revision: 0 fatal, 0 errors, 21 warnings (supplier-attribute mismatch + intentionally unused pins).

---

## 17. Revision — production readiness (2026-10-03)

| Item | Page | Change | Designators |
|------|------|--------|-------------|
| Fix | CPU_part | VDDA/VREF+ net was split (`Vref+` vs `VREF+`), so FB1 did not feed VDDA. All wires renamed to `VREF+`; FB1 now supplies VDDA/VREF+ | FB1, C5/C7/C8/C9 |
| SWD | CPU_part / Watchdog | `SWDIO` (PA13), `SWCLK` (PA14) labels; Cortex 10-pin 1.27 mm SMD header, **DNP in production**, for factory recovery. Pin 6 SWO unused (PB3 is EXP_SCK) | J903 (FTSH-105) |
| Test points | Watchdog | +24V_PROT, +5V, VCC, VREF+, GND, NRST, STM_TX, STM_RX | TP901–TP908 (RH-5015) |
| Mounting | Watchdog | 4 × M3 SMT standoffs, all pads bonded to `PE` | H1–H4 (SMTSO-M3) |
| Latch memory | Flash_memory | W25Q64 replaced by **FM25V20A-GTR 2 Mbit SPI FRAM** (same SOIC-8 208 mil pinout, same SPI2/LATCH_CS nets). No erase, 10^14 writes, a full latch image writes in < 1 ms → fits easily in the ~10–15 ms hold-up | U5 |
| Expansion ESD | Expansion bus | 4 × TPD4E05U06 on all 13 bus signals (SCK, MOSI, MISO, ATTN, CS0–7, SYNC); SMBJ33A on +24V_EXP, SMAJ5.0A on +5V_EXP | U1002–U1005, D1001, D1002 |

DRC after this revision: 0 fatal, 0 errors, 21 warnings (supplier-attribute mismatch, intentionally unused pins).

Firmware impact: replace the W25Q64 driver in `ModbusRTU_AWS` with an FRAM driver (opcodes WREN 0x06 / WRITE 0x02 / READ 0x03, 3-byte address, no busy polling, no erase). Board temperature is available from the DS3231 temperature register (0x11/0x12) — no extra sensor needed.

## 18. Roadmap — industrial grade and market differentiators

### Industrial-grade upgrades (hardware)

| Priority | Upgrade | Why |
|---|---|---|
| 1 | Replace ULN2803 DO stage with protected high-side switches (e.g. TPS274160 / VNI8200XP / ISO1H811G) | Short-circuit, over-temperature and open-load diagnostics per channel; sourcing (PNP) outputs are the EU/Asia standard |
| 1 | DI front end to IEC 61131-2 Type 1/3 with current limiting (e.g. MAX22190 / CLT01-38SQ7) | Defined thresholds, wire-break detection, lower heat than resistor dividers |
| 1 | Surge to IEC 61000-4-5 on 24 V input: SMCJ/5.0SMDJ TVS + slow-blow fuse, inrush limiter | CE/EMC compliance; protects against cabinet transients |
| 2 | AI: 16-bit ADC with per-channel open-wire/over-range detection, optional isolation (e.g. AD4111) | 4–20 mA wire-break and NAMUR NE43 diagnostics |
| 2 | Secure element (ATECC608B / OPTIGA Trust M) on I2C for AWS IoT X.509 keys | Keys cannot be extracted from flash; required for IEC 62443-4-2 / EU CRA |
| 2 | Conformal coating, -40…+85 °C BOM audit, 2 mm creepage on field side, pluggable terminals | Long life in humid / dusty cabinets |
| 3 | Isolated RS-232 and isolated AI groups | Ground-loop immunity in large plants |
| 3 | Type tests: EN 61131-2, IEC 61000-6-2/-4, UL 61010-2-201 | Required for selling to OEMs |

### Innovations that add market value

1. **Zero-tool commissioning**: NFC dynamic tag (ST25DV04K on I2C) — configure IP/Modbus ID/cloud keys with a phone, even unpowered; plus BLE provisioning on the ESP32.
2. **Built-in web HMI + REST API** served by the ESP32 (later Linux): live I/O view, forcing, logs, firmware update — no SCADA licence needed for small sites.
3. **Edge analytics / predictive maintenance**: pulse counters, run-hour meters, cycle counts and AI trend alarms computed on the device and pushed to the existing ThingsBoard dashboard.
4. **Secure OTA for both MCUs** through the ESP32 → STM32 ROM bootloader path already in hardware; signed images, rollback.
5. **Protocol gateway mode**: Modbus RTU master on RS-485 + Modbus TCP server + MQTT/Sparkplug B — sells as a retrofit gateway for legacy machines.
6. **Self-diagnostics telemetry**: 24 V supply level (POWER_FAIL ADC), board temperature (DS3231), watchdog resets, DO short-circuit counters → "health score" per device in the cloud.
7. **IEC 61131-3 programming** (OpenPLC runtime on the STM32 or CODESYS on the v2 Linux head) so automation engineers can use ladder/ST instead of C.
8. **Modular I/O** over the expansion bus with auto-discovery (each module reports ID/serial) — one head, many SKUs.

### Ethernet and Linux

See `docs/ETHERNET_UPGRADE.html` (W5500 on ESP32-S3 for v1.1, dual-port on v2) and the "IoT PLC — Embedded Linux SoC Selection" doc (STM32MP135F recommended).

---

## 19. Revision — Ethernet uplink page (2026-10-03)

New schematic page **10. Ethernet**: W5500 10/100 Ethernet controller on the ESP32-S3, for the SCADA link and Internet access (Wi-Fi stays as fallback route).

| Function | Implementation | Designators |
|---|---|---|
| Controller | W5500 (MAC + PHY, SPI up to 80 MHz, LQFP-48, −40…+85 °C) | U1101 |
| ESP32-S3 link | FSPI IOMUX pins: IO10 `ETH_CS`, IO11 `ETH_MOSI_ESP`→22 Ω→`ETH_MOSI`, IO12 `ETH_SCLK_ESP`→22 Ω→`ETH_SCLK`, IO13 `ETH_MISO`, IO9 `ETH_INT`, IO14 `ETH_RST` (10k pull-ups on CS/INT/RST, 100 nF on RST) | R1109–R1113, C1113 |
| Analog supply | `3V3_ETH` from VCC through FB1101, 10 µF + 3 × 100 nF; digital VDD on VCC + 100 nF | FB1101, C1101–C1105 |
| PHY support | EXRES1 12.4k 1 %, TOCAP 4.7 µF, 1V2O 10 nF, 25 MHz crystal + 2 × 20 pF, PMODE open (auto-negotiation) | R1115, C1108, C1109, Y1101, C1110, C1111 |
| Line interface | 49.9 Ω 1 % terminations (TX to 3V3_ETH, RX to 6.8 nF AC node), magnetics centre taps to 3V3_ETH | R1101–R1104, C1107 |
| Jack | J1B1211CCD magjack (WIZnet W5500-io reference pinout: TD+1, TCT2, TD−3, RD+4, RCT5, RD−6, 8 = Bob-Smith/CHGND). LED anodes 10/12 → VCC, cathodes 9/11 via 330 Ω → ACTn/LINKn | J1101, R1107, R1108 |
| Protection | TPD4E05U06 on TX±/RX± (PHY side); shield + Bob-Smith node `ETH_SHIELD` → PE via 1 nF/2 kV ∥ 1 MΩ | U1102, C1112, R1114 |

Jack pinout and LED wiring follow the WIZnet W5500-io reference schematic. TCT/RCT tied to 3V3_ETH (W5500 current-mode driver) — confirm link at 100 Mb/s on the first prototype.

**Firmware architecture for SCADA / Internet (IoTHandler):**
- ESP-IDF `esp_eth` W5500 driver in **MACRAW mode** under lwIP (not the W5500's 8 hardware sockets) → TLS, many concurrent connections.
- Clients/servers on the same stack: MQTT over TLS (AWS IoT / Sparkplug B), **OPC UA** (open62541, client and/or server, buffers in the 8 MB PSRAM), Modbus TCP server/client (port 502), HTTPS web UI + REST.
- Routing: Ethernet default route, Wi-Fi fallback; SNTP + DS3231 for timestamps; per-interface firewall (SCADA port only on the plant network).
- Throughput expectation: about 10–15 Mbit/s in MACRAW at 40 MHz SPI — far above SCADA polling needs.

---

## 20. Revision — prototype release v1.0 (2026-10-07)

Schematic review findings closed:

| # | Change | Designators |
|---|--------|-------------|
| 1 | ESP32-S3 BOOT button on IO0 (`ESP_BOOT`) + UART0 test points (`ESP_TXD0`, `ESP_RXD0`) — recovery if USB firmware breaks | SW3, TP909, TP910 |
| 2 | LCSC supplier part numbers set on all BOM parts (was a library placeholder on 413 parts); missing values filled | all |
| 3 | Ethernet jack changed to J1B1211CCD and wired exactly like WIZnet W5500-io reference (LEDs, centre taps, CHGND) | J1101 |
| 4 | 10 k pull-down on `RS485_DE` — transceiver stays in receive until firmware runs | R802 |
| 5 | HSE crystal load caps 22 pF → 12 pF (X1 needs CL = 10 pF) | C13, C14 |
| 6 | AI current shunts → 150 Ω 0.1 % thin-film 1206 (0.25 W) for 24 V mis-wiring | R668, R674, R680, R686 |
| 7 | AO supply PTC 50 mA → 200 mA / 48 V | F701 |
| 8 | Input fuse → 2 A slow-blow 125 V (0452002.MRL) for capacitor inrush | F501 |
| 9 | SWD header now fitted on prototypes (first STM32 firmware load) | J903 |
| 10 | Power LED resistors 100 Ω / 220 Ω → 1 kΩ | R23, R24 |
| 11 | 4.7 µF bulk capacitor at the STM32 | C17 |
| 12 | Mounting: Würth 9774030151R M3 SMT standoffs, bonded to PE | H1–H4 |

Status: DRC 0 fatal / 0 errors; remaining warnings are intentionally unused pins. Netlist re-checked: no single-pin nets.

Assembly notes:
- BT401 (CR2032 with solder pins) is excluded from the JLC BOM — hand-solder after assembly.
- Not changed (by decision): USB VBUS does not power the board (back-feeding the LM5160 and 290 µF from USB would exceed USB inrush limits) — bench programming needs 24 V.
- 5 V budget: LM5160 1.5 A max; keep +5V_EXP load ≤ 300 mA on the first prototype and measure.
- Analog current inputs: a sustained 24 V fault still puts ≈0.3 W in the shunt — acceptable for prototype; v2 should add a 30 mA current limiter per channel.
