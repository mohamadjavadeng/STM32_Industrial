# IoTHandler — ESP32-S3 network half of the IIoT gateway

Counterpart to `../ModbusRTU_AWS` (STM32F407ZGT6). The STM32 is the industrial
half: Modbus RTU master on RS-485, owner of the process data and the IO. This
chip does connectivity only — WiFi, TLS, MQTT to AWS IoT Core, and the downlink
command path.

The two talk over a UART using the `0xAA`/`0xBB` framed protocol implemented in
`ModbusRTU_AWS/Core/Src/esp32msghandler.c`.

---

## Wiring

| ESP32-S3 | direction | STM32F407 | note |
|---|---|---|---|
| GPIO18 (`PIN_STM_RX`) | ← | PB10 / USART3_TX | |
| GPIO17 (`PIN_STM_TX`) | → | PB11 / USART3_RX | |
| GND | — | GND | mandatory, keep it short |
| GPIO4 (`PIN_STM_EVENT`) | ← | any GPIO | optional, not driven by STM32 V1.2 |
| GPIO5 (`PIN_STM_RESET`) | → | NRST | optional, open-drain |

115200 8N1 — must match `huart3.Init.BaudRate` in the STM32's `main.c`.

Pins were picked to avoid GPIO 0/45/46 (strapping), 19/20 (native USB), 26–37
(octal flash + PSRAM on the N8R8 module), 43/44 (UART0 debug console) and 48
(RGB LED). `Serial` is the debug console on GPIO43/44; **never log to `Serial1`**,
that is the STM32 link.

If the industrial side is mains-referenced, put a digital isolator in the two
data lines. A 2-wire UART is the cheapest thing to isolate — that is one of the
reasons to prefer it over SPI here.

---

## Build

```sh
# 0. NEW: bring up the v2 inter-chip link first. No WiFi, no libraries,
#    no certificates. Procedure: ../docs/LINK_V2_TESTING.html
pio run -e linkv2 -t upload -t monitor

# 1. Legacy v1 (0xAA/0xBB) link test.
pio run -e linktest -t upload -t monitor

# 2. Then the full gateway.
cp include/secrets.h.example include/secrets.h    # then fill it in
pio run -e freenove_esp32_s3_wroom -t upload -t monitor
```

The first build of the full environment downloads ArduinoJson and PubSubClient,
so it needs network access. `linktest` has no library dependencies.

Both environments currently build warning-clean under `-Wall`:
`linktest` 6.1 % RAM / 8.6 % flash, full 15.3 % / 26.5 %.

---

## Layout

| file | role |
|---|---|
| `include/config.h` | pins, baud, timeouts, intervals, feature flags |
| `include/gw_model.h` | shim onto `../../shared/gw_model.h`, the **v2** wire contract |
| `include/stm_link_v2.h` | **v2** client: transactions over the STM32 process image |
| `src/stm_link_v2.cpp` | framing, sequence numbers, retries, block reads, stats |
| `src/linkv2_test_main.cpp` | v2 bring-up console — its own firmware, env `linkv2` |
| `include/stm_protocol.h` | v1 wire format + the catalogue of STM32 V1.2 quirks |
| `src/crc16_modbus.cpp` | CRC-16/MODBUS, verified identical to the STM32's table |
| `src/stm_link.cpp` | framed transaction master, retries, resync, stats |
| `src/tag_map.cpp` | **the table you edit** — what to poll and publish |
| `src/process_image.cpp` | cached values + quality + age, mutex protected |
| `src/cloud_client.cpp` | WiFi, SNTP, TLS, MQTT, JSON, downlink parsing |
| `src/main.cpp` | task wiring, poll engine, command execution, supervisor |

Tasks: `link` on core 1 owns `Serial1` and nothing else; `net` on core 0 runs the
TLS/MQTT stack; they exchange fixed-size structs over two FreeRTOS queues. No
shared mutable state beyond the process image and the link stats, both locked.

---

## Two link protocols

**v2 (`0xA5` / `0x5A`) is the one to use.** It reads bytes out of a process image
on the STM32 instead of proxying a live Modbus transaction, which is what lets the
response timeout drop from 1600 ms to 100 and lets quality travel with every value.
The contract is `../shared/gw_model.h`, compiled by both chips; the STM32 side is
`ModbusRTU_AWS/Core/Src/gw_link.c`. Bring-up: `../docs/LINK_V2_TESTING.html`.

```
request   A5 seq op  reg addrLo addrHi lenLo lenHi [payload...] crcLo crcHi
response  5A seq status reg lenLo lenHi           [payload...]  crcLo crcHi
```

**v1 (`0xAA` / `0xBB`) is the original**, kept for reference and still selectable
by building the STM32 with `GW_LINK_ENABLE 0`. Only one protocol can own USART3.

```
request   AA cmd type addrHi addrLo count [data...] crcLo crcHi
response  BB status len   [data...]                 crcLo crcHi
```

CRC-16/MODBUS (poly `0xA001` reflected, init `0xFFFF`) over everything before it,
**low byte first**. Requests have no length field — the STM32 infers
`6 + (cmd == WRITE ? count : 0) + 2`. Responses do carry a payload length at byte
2, which is what lets one parser handle every reply shape.

| cmd | name | request | response |
|---|---|---|---|
| `0x01` | READ | 8 bytes | `BB 00 02 hi lo crc` (7) |
| `0x02` | WRITE | 10 bytes, `count = 2` | `BB 00 00 crc` (5) |
| `0x03` | MULREAD | 8 bytes | `BB 00 count <count> crc` |
| `0x04` | MULWRITE | — | unimplemented on the STM32, never send |

Register classes: `0x01` holding, `0x02` input, `0x03` coil, `0x04` discrete.
Status: `0x00` OK, `0x01` error.

### Verified test vectors

Paste these into a serial terminal to exercise the STM32 without the ESP32:

```
ESP32 -> STM32
  READ holding 4097      AA 01 02 10 01 01 E5 FC     (type 0x02 — see Q1)
  READ input   4097      AA 01 01 10 01 01 E5 B8     (type 0x01 — see Q1)
  READ coil    1281      AA 01 03 05 01 01 F5 C4
  READ discrete 1281     AA 01 04 05 01 01 F4 B0
  WRITE holding 4097=823 AA 02 01 10 01 02 03 37 C8 C4
  WRITE coil 1281=ON     AA 02 03 05 01 02 00 01 44 03
  MULREAD 4 regs @4097   AA 03 01 10 01 08 5C 7E     (count = 2 × regs — see Q2)

STM32 -> ESP32
  READ rsp, value 1234   BB 00 02 04 D2 E3 46
  WRITE ack              BB 00 00 01 E5
  ERROR (bad request CRC) BB 01 00 00 75
```

### STM32 V1.2 quirks this firmware works around

Full detail in `include/stm_protocol.h`. Guarded by `STM_FW_QUIRKS` — set it to 0
once the STM32 side is fixed and the workarounds disappear.

- **Q1** Single reads have holding and input swapped. `ESP32MsgHandler_ReadRegister`
  sends `REG_TYPE_HOLDING` to `Modbus_ReadInputRegisters` and vice versa, so to
  read a holding register with cmd `READ` you must put `0x02` in the type byte.
  `MULREAD` does *not* have the swap, so the type byte means different things in
  cmd `0x01` and cmd `0x03`.
- **Q2** `count` is response bytes on the link but register quantity on the field
  bus. Asking for N registers means sending `count = 2N`, and the STM32 will
  request 2N registers from the RS-485 slave while returning only N. Cap
  `count ≤ 100` (`uint16_t holdingRegisters[100]`).
- **Q3** `MULREAD` only works for holding registers. The input branch truncates
  uint16 into uint8; the coil/discrete branches treat packed Modbus bitmask bytes
  as one-byte-per-bit and are off by three.
- **Q4** A failed `MULREAD` leaves the STM32 parser holding the frame
  (`continue` at `esp32msghandler.c:186` skips the `rxIndex` reset), so the next
  byte re-executes the previous request. `StmLink::resync()` deliberately feeds
  one dummy byte to flush that ghost.
- **Q5** **`STATUS_OK` does not mean the data is valid.** `statusModbus` is stored
  in a global and never transmitted, so a read whose RS-485 transaction timed out
  still answers `BB 00 02 <stale bytes>`. This is why `ProcessImage` tracks
  quality from timestamps and why writes are verified by read-back
  (`WRITE_VERIFY`). It cannot be fully fixed from this side.
- **Q6/Q7** `MULWRITE` is unimplemented; a `WRITE` with `count < 2` is dropped
  with no reply.
- **Q8** The STM32 receive buffer is 64 bytes with no bounds check on `rxIndex`,
  so a frame claiming `count > 56` overflows it. Requests here are capped, but
  line noise can still trigger it — a reason to fix the STM32, not to rely on a
  well-behaved client.

### Timing constraint

The STM32 answers a request by running a **blocking** Modbus transaction
(`HAL_MAX_DELAY` transmit, 1000 ms receive timeout in `Modbus_SendRequest`), and
it services the link by polling one byte at a time from its main loop. So:

- per-request timeout must exceed 1000 ms (`STM_RSP_TIMEOUT_MS` is 1600),
- never pipeline — one request in flight, always,
- a dead field slave costs 1.6 s of link time per affected tag.

---

## MQTT interface

Topics are prefixed with `DEVICE_ID` from `secrets.h`.

`<id>/telemetry` — every `TELEMETRY_INTERVAL_MS`:

```json
{"dev":"iiot-gateway-01","ts":1723545600,"seq":42,
 "tags":[{"n":"level_pct","v":37.5,"raw":3750,"q":"good","age":420,"u":"%"}]}
```

`<id>/status` — diagnostics, also the last-will payload: heap, RSSI, IP, and the
full link counters (requests, timeouts, CRC errors, retries, resyncs, RTT).

`<id>/cmd` — downlink. `id` is echoed in the ack so callers can correlate:

```json
{"id":"c1","op":"write","tag":"pump_run","value":1}
{"id":"c2","op":"write","tag":"level_pct","value":42.5}
{"id":"c3","op":"write","tag":"hold_4097","value":823,"raw":true}
{"id":"c4","op":"writeRaw","type":1,"addr":4097,"value":823}
{"id":"c5","op":"ping"}
```

`<id>/cmd/ack`:

```json
{"dev":"iiot-gateway-01","ts":...,"id":"c1","ok":true,"accepted":true,
 "status":"ok","tag":"pump_run","raw":1}
```

`accepted:false` means it was rejected before reaching the wire (unknown tag,
read-only tag, value out of range). `ok:false` with `accepted:true` means the
transaction ran and failed — `status` says how, and `detail` is
`"read-back mismatch"` when the write was acknowledged but did not stick.

AWS IoT accepts no retained messages, so nothing here is published retained.

---

## Status LED

RGB on GPIO48, pulsing once per second.

| colour | meaning |
|---|---|
| green | STM32 link answering, MQTT connected |
| blue | link fine, cloud not connected (or `ENABLE_CLOUD=0`) |
| red | STM32 has not answered recently |

---

## Recommended next step on the STM32 side

The largest remaining constraint is not in this firmware — it is that the STM32
performs a live RS-485 transaction inside the ESP32 request path. Moving the
STM32 to a free-running scanner plus a RAM process image would make link replies
immediate and deterministic, decouple cloud poll rate from field-bus health, and
retire Q1–Q3 and Q5 outright. See `../docs/GATEWAY_ARCHITECTURE.md`.
