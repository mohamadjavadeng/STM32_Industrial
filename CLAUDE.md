# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository shape

This repo is an **STM32CubeIDE (Eclipse) workspace**, not a plain source tree. The workspace metadata (`.metadata/`) and the generated build output (`ModbusRTU_AWS/Debug/`, including `.o`/`.elf`/`.map`/`.list`) are **tracked in git** — there is no `.gitignore`. Any IDE session or build dirties the working tree with thousands of lines of churn. Keep functional diffs separate from that noise when committing.

Two projects, the two halves of one IIoT gateway:

- `ModbusRTU_AWS/` — **industrial half**, STM32F407ZGT6 (LQFP144), Cortex-M4F, hard float, `STM32Cube FW_F4 V1.28.2`, STM32CubeIDE.
- `IoTHandler/` — **network half**, ESP32-S3 (Freenove WROOM N8R8), Arduino framework, PlatformIO. Has its own `README.md`.

`docs/GATEWAY_ARCHITECTURE.md` covers the two-chip split: why UART rather than SPI, why the ESP32 initiates and the STM32 responds, the recommended process-image refactor, and a failure-mode table. Read it before changing anything about the inter-chip link.

## Build / flash

No `make` or `arm-none-eabi-gcc` on PATH. Everything lives inside the CubeIDE install:

- IDE: `C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\`
- Headless build: `C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\headless-build.bat -data <repo-root> -build ModbusRTU_AWS/Debug` (use `-cleanBuild` to force a full rebuild)
- Toolchain bin: `C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin`
- Build configurations defined in `.cproject`: `Debug` and `Release` (only `Debug` has ever been generated/committed)

**The committed `Debug/makefile` cannot be used as-is.** Lines 63–64 hardcode the linker script as `C:\Users\Mohamad\Documents\GitHub\STM32_Industrial\ModbusRTU_AWS\STM32F407ZGTX_FLASH.ld` — a path from the machine that generated it. Direct `make` fails with a missing-target error. Either build through CubeIDE (it regenerates `Debug/**/*.mk` and `Debug/makefile`), or rewrite those two occurrences to the local repo path first. Any such edit is disposable — CubeIDE overwrites it.

Toolchain version skew to be aware of: the committed makefiles say *"Toolchain: GNU Tools for STM32 (12.3.rel1)"*; the installed IDE ships 14.3.rel1. First IDE build regenerates them for 14.3.

Compile flags (from `Debug/Core/Src/subdir.mk`): `-mcpu=cortex-m4 -std=gnu11 -O0 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -Wall -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb --specs=nano.specs`.

Debug/flash: `ModbusRTU_AWS Debug.launch` → ST-LINK, SWD, live expressions on, `Debug/ModbusRTU_AWS.elf`.

**There are no tests, no linter, and no CI.** Verification is compile + on-hardware.

## IoTHandler (ESP32-S3) build

Normal PlatformIO project; `pio` is at `C:\Users\Asus\.platformio\penv\Scripts\pio.exe` (not on PATH). Installed: platform espressif32 7.0.0, Arduino core **2.0.17** (not 3.x — mind the API differences).

```sh
pio run -e linktest                  # no WiFi/TLS, no credentials, no lib_deps
pio run -e freenove_esp32_s3_wroom   # full gateway; needs include/secrets.h
```

`include/secrets.h` is gitignored; `secrets.h.example` is the template, and a `__has_include` guard emits an actionable `#error` when it is missing. Bring the link up with `linktest` before touching certificates.

Two gotchas that cost real debugging time on this core:

- `HardwareSerial::flush()` with no argument calls `uart_flush_input()` and **discards the RX buffer**. Use `flush(true)` to wait for TX only. `StmLink` depends on this.
- `Serial` is UART0 on GPIO43/44 (the board sets `ARDUINO_USB_CDC_ON_BOOT=0`), and `Serial1` is the STM32 link. Never log to `Serial1`.

The wire protocol, its verified test vectors, and the catalogue of STM32 V1.2 quirks the client works around live in `IoTHandler/include/stm_protocol.h` and `IoTHandler/README.md`. `STM_FW_QUIRKS` in `include/config.h` switches the workarounds off once the STM32 side is fixed.

## CubeMX regeneration contract

`ModbusRTU_AWS.ioc` is the source of truth for peripheral init. Regenerating from it rewrites `main.c`, `stm32f4xx_hal_msp.c`, `stm32f4xx_it.c`, `main.h`, and the `Debug/**` makefiles, **preserving only text between `/* USER CODE BEGIN X */` and `/* USER CODE END X */`**. Never add application code outside those markers in generated files.

Hand-written files (`modbusMaster.[ch]`, `esp32msghandler.[ch]`, `modbus_crc.[ch]`) live in `Core/Src` and `Core/Inc` and survive regeneration, but their entries in `Debug/Core/Src/subdir.mk` are regenerated — a new hand-written `.c` only enters the build after CubeIDE re-scans the project.

## Architecture: ESP32 ⇄ Modbus RTU bridge

The firmware is a protocol translator with two UARTs and no RTOS. `main()` initializes and then spins on a single call:

```
while (1) { ESP32MsgHandler_Task(); }
```

Data flow:

```
ESP32  --USART3 (115200, PB10/PB11)-->  esp32msghandler.c  -->  modbusMaster.c  --USART1 (9600, PA9/PA10, DE=PA11)-->  RS-485 slave
```

Three layers, bottom up:

1. **`modbus_crc.c`** — table-driven Modbus CRC16 (`crc16()`), the *only* CRC in the project. It is shared by both protocols: the RS-485 Modbus frames and the custom ESP32 frames. (`esp32msghandler.c` still carries a commented-out bitwise duplicate; ignore it.)

2. **`modbusMaster.c` / `ModbusMaster` handle** — blocking Modbus RTU master. All public functions build a frame in `modbus->txBuffer`, then funnel through the private `Modbus_SendRequest(modbus, requestLength, expectedResponseLength)`, which raises the RS-485 DE pin, does a blocking `HAL_UART_Transmit(..., HAL_MAX_DELAY)`, drops DE, blocking-reads exactly `expectedResponseLength` bytes with a 1000 ms timeout, and verifies CRC. Response length is *computed by the caller from the request*, never parsed from the reply. Errors surface as `ModbusStatus` (`MODBUS_OK`/`ERROR`/`TIMEOUT`/`INVALID_CRC`/`INVALID_RESPONSE`).

3. **`esp32msghandler.c`** — the bridge. It byte-scans USART3 and, for each valid frame, calls one of three user-implementable hooks (`ESP32MsgHandler_ReadRegister`, `_MultipleReadRegister`, `_WriteRegister`) which are implemented *in the same file* as Modbus master calls against the globals `hmodbus` and `SlaveID` (both `extern`, defined in `main.c`; `SlaveID` is hardcoded to 2).

### ESP32 wire protocol (defined only in code — `esp32msghandler.h`)

Request: `0xAA | cmd | regType | addrHi | addrLo | count | [data…] | crcLo | crcHi`
Response: `0xBB | status | byteCount | [data…] | crcLo | crcHi`

- `cmd`: `0x01` READ, `0x02` WRITE, `0x03` MULREAD, `0x04` MULWRITE *(0x04 is defined but not handled)*
- `regType`: `0x01` HOLDING, `0x02` INPUT, `0x03` COIL, `0x04` DISCRETE
- `status`: `0x00` OK, `0x01` ERROR
- CRC is little-endian on the wire (low byte first) in both directions, for both protocols.
- Framing is length-implied: `packetLen = 6 + (cmd==WRITE ? count : 0) + 2`. There is no length field in the header beyond `count`.

### Peripheral facts worth knowing

- SYSCLK 168 MHz from a **25 MHz HSE** (PLLM 25 / PLLN 336 / PLLP 2), APB1 42 MHz, APB2 84 MHz, flash latency 5.
- **DMA is fully configured but completely unused.** `.ioc` sets up four streams (USART1 RX/TX on DMA2 S2/S7, USART3 RX/TX on DMA1 S1/S3), `HAL_UART_MspInit` calls `__HAL_LINKDMA`, and all six IRQ handlers exist in `stm32f4xx_it.c` — yet every transfer in application code is blocking `HAL_UART_Transmit`/`HAL_UART_Receive`. Converting to DMA/interrupt I/O needs no CubeMX change, only application rewrites.
- **SPI2 (PB13/14/15) is initialized and never used.**
- GPIO outputs: `RS485_EN` = PA11 (`main.h`), plus PC4 and PD12–PD15 (LEDs) driven nowhere in current code.

## Known rough edges in the current code

Do not "tidy" these silently — they are live behavior; confirm intent before changing.

- `ESP32MsgHandler_ReadRegister` (`Core/Src/esp32msghandler.c:44`) maps `REG_TYPE_HOLDING` → `Modbus_ReadInputRegisters` and `REG_TYPE_INPUT` → `Modbus_ReadHoldingRegisters` — swapped relative to `ESP32MsgHandler_MultipleReadRegister`, which maps them straight.
- `ESP32MsgHandler_MultipleReadRegister` uses different, mutually inconsistent index arithmetic per branch (`buff[i+3]/buff[i+4]` with `i += 2` for HOLDING; `buff[i]/buff[i+1]` starting at `i = 3` for INPUT, which also truncates `uint16_t` registers into `uint8_t` slots), and loop bounds are `quantity` rather than a byte count.
- `count` is treated as a *register quantity* when passed to the Modbus layer but as a *byte count* when sizing the response (`response[2] = count`, CRC over `count + 3`, transmit `count + 5`).
- The MULREAD error path does `continue` **without** resetting `rxIndex`, so a failed multi-read leaves the frame in the buffer and it is re-parsed on the next byte.
- `statusModbus` is `uint8_t` but holds `ModbusStatus`; `ESP32MsgHandler_WriteRegister` returns that status directly, so a Modbus failure is reported as a nonzero value the caller ignores.
- `MODBUS_EXCEPTION_MASK` (0x80) is defined but never checked — a slave exception reply is a short frame and surfaces as `MODBUS_TIMEOUT`/`MODBUS_INVALID_CRC`, not `MODBUS_INVALID_RESPONSE`.
- `responseLength` is `uint8_t` in `Modbus_ReadHoldingRegisters` / `Modbus_ReadInputRegisters`, so `quantity > 125` overflows (only `ReadInputRegisters` range-checks it).
- `Core/Src/Test.txt` is **not code** — it is a scratch backup of an earlier `esp32msghandler.c` (register arrays served from local memory, no Modbus) plus a retired `main()` loop. It is stale and does not compile; do not treat it as reference. It does record the real slave register offsets used in bench testing (holding `4097`, coils `1281`, slave ID 2).
- The `holdingRegisters` / `inputRegisters` / `coils` / `discreteInputs` arrays in `main.c` are no longer a register model — they are now just scratch landing buffers for Modbus reads, though `main()` still seeds two of them with test values.
