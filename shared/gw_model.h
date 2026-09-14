/**
 * gw_model.h - the single wire contract between the two halves of the gateway.
 * ---------------------------------------------------------------------------
 *
 * WHO COMPILES THIS FILE
 *   - STM32F407 (ModbusRTU_AWS) via the shim Core/Inc/gw_model.h
 *   - ESP32-S3  (IoTHandler)    via the shim include/gw_model.h
 *
 *   Both shims are two lines long and do nothing but #include this file with a
 *   relative path, so neither project needs an extra include directory and
 *   neither build system has to be touched. There is exactly one copy of the
 *   protocol in the repository: this one.
 *
 * THE RULE
 *   Never change a constant here for one side only. A mismatch between the two
 *   halves of a binary protocol is silent - frames still parse, values are just
 *   wrong. If you add an opcode, a region or a status code, add it here, then
 *   rebuild and reflash BOTH chips, and bump GW_PROTO_VERSION.
 *
 * WHAT THIS PROTOCOL IS FOR (and what it deliberately is not)
 *   Link v2 moves *bytes out of a process image*. It knows nothing about Modbus,
 *   CAN, slave IDs, register numbers, tag names, MQTT or clouds. The STM32 owns
 *   the field buses and writes their results into a RAM image; the ESP32 asks
 *   for a slice of that image and publishes it.
 *
 *   That indirection is the whole point of the design:
 *     - Changing an RS-485 slave ID, or moving a tag from Modbus RTU to CAN,
 *       changes the STM32 config only. No ESP32 rebuild, no protocol change.
 *     - A dead field device costs one slot's quality flag. It can no longer
 *       stall the link, because the link never touches a field bus.
 *     - The same three cloud back ends (AWS IoT, ThingsBoard, plain MQTT) sit
 *       on top of the same image. The cloud layer never learns a new protocol
 *       when a new bus is added.
 *
 *   Version 1 of the link (0xAA / 0xBB, in esp32msghandler.c) was a synchronous
 *   proxy for Modbus: an ESP32 read triggered a live RS-485 transaction. Every
 *   known defect of the old link - the swapped register classes, the
 *   byte-versus-register count muddle, the parser desync, and worst of all a
 *   failed field read reported to the cloud as STATUS_OK with stale bytes -
 *   comes from that one decision. v2 exists to undo it.
 *
 * DESIGN NOTES THAT ARE EASY TO GET WRONG LATER
 *   - Every multi-byte field on the wire is LITTLE ENDIAN, including the CRC.
 *     (Values inside the image are little endian too - both MCUs are LE, so
 *     nothing is ever byte swapped anywhere in the system.)
 *   - The request header is self-describing: the frame length is
 *     GW_REQ_HEADER_LEN + LEN + GW_CRC_LEN for *every* opcode, with no
 *     per-opcode special case. This is the single biggest difference from v1
 *     and the reason one parser can handle every frame shape.
 *   - CRC-16/MODBUS is reused unchanged from the RS-485 side (poly 0xA001
 *     reflected, init 0xFFFF, no final xor, low byte first). Reusing it means
 *     no new code to test: both implementations are already proven against
 *     each other, check value 0x4B37 for ASCII "123456789".
 *
 * Companion documents:
 *   docs/GATEWAY_ARCHITECTURE.md      - why the chips are split the way they are
 *   docs/UNIVERSAL_PROCESS_IMAGE.html - the full process-image plan (this file
 *                                       implements its phases P2 and P4)
 *   docs/LINK_V2_TESTING.html         - how to bring this up on the bench
 */
#ifndef GW_MODEL_H
#define GW_MODEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 1. Versioning
 * ==========================================================================
 * GW_PROTO_VERSION is returned by SYNC and RD_META. The ESP32 refuses to talk
 * to an STM32 whose major version it does not know, which turns "someone
 * flashed a mismatched pair" from a silent data-corruption bug into a loud
 * startup error. Bump the major on any incompatible change (field moved, field
 * resized, opcode reused); bump the minor when you only add things.
 */
#define GW_PROTO_VERSION_MAJOR 2
#define GW_PROTO_VERSION_MINOR 1
#define GW_PROTO_VERSION ((GW_PROTO_VERSION_MAJOR << 4) | GW_PROTO_VERSION_MINOR)

/* ==========================================================================
 * 2. Framing
 * ==========================================================================
 *
 * REQUEST  (ESP32 -> STM32), header is 8 bytes:
 *
 *   off  field   size  meaning
 *   ---  ------  ----  -------------------------------------------------------
 *    0   SOF      1    always 0xA5
 *    1   SEQ      1    1..255, wraps, never 0. Echoed in the response so a late
 *                      reply can never be mistaken for the answer to the next
 *                      request. SEQ 0 disables both echo checking and the
 *                      duplicate-retry cache on the STM32.
 *    2   OP       1    GW_OP_*
 *    3   REG      1    region id, or an opcode-specific argument
 *    4-5 OFF      2    byte offset inside the region (LE), or an opcode
 *                      specific argument
 *    6-7 LEN      2    number of PAYLOAD bytes that follow this header (LE)
 *    8.. payload  LEN
 *   last CRC      2    CRC-16/MODBUS over every preceding byte, low byte first
 *
 * RESPONSE (STM32 -> ESP32), header is 6 bytes:
 *
 *    0   SOF      1    always 0x5A
 *    1   SEQ      1    echo of the request's SEQ
 *    2   STATUS   1    GW_ST_* - a real result code, not "I parsed your frame"
 *    3   REG      1    echo of the request's REG
 *    4-5 LEN      2    number of PAYLOAD bytes that follow (LE)
 *    6.. payload  LEN
 *   last CRC      2    CRC-16/MODBUS over every preceding byte, low byte first
 *
 * WHY LEN COUNTS PAYLOAD ONLY, FOR EVERY OPCODE
 *   A read has to say how many bytes it wants. The obvious move is to overload
 *   LEN ("read length on reads, payload length on writes"), and the plan
 *   document sketches it that way. Do not. The moment LEN means different
 *   things per opcode, the parser has to know the opcode to find the end of the
 *   frame, and an unknown or corrupted opcode leaves it unable to resynchronise
 *   - which is precisely how v1 wedged itself. Here a read carries its count as
 *   a 2-byte payload (GwReadArgs) and costs two extra bytes on the wire. Two
 *   bytes is a very cheap price for a parser that never needs to look past the
 *   header to know where a frame ends.
 *
 * WHY THERE IS NO END-OF-FRAME BYTE
 *   In a binary protocol no byte value can be reserved as a terminator, because
 *   the payload may legitimately contain it. LEN plus CRC already defines the
 *   boundary exactly. An EOF byte would cost a byte and buy false confidence.
 */
#define GW_SOF_REQ 0xA5u
#define GW_SOF_RSP 0x5Au

#define GW_REQ_HEADER_LEN 8u
#define GW_RSP_HEADER_LEN 6u
#define GW_CRC_LEN 2u

/* Field offsets, so no parser anywhere uses a magic number. */
#define GW_REQ_OFF_SOF 0u
#define GW_REQ_OFF_SEQ 1u
#define GW_REQ_OFF_OP 2u
#define GW_REQ_OFF_REG 3u
#define GW_REQ_OFF_OFFSET 4u /* 2 bytes, LE */
#define GW_REQ_OFF_LEN 6u    /* 2 bytes, LE */

#define GW_RSP_OFF_SOF 0u
#define GW_RSP_OFF_SEQ 1u
#define GW_RSP_OFF_STATUS 2u
#define GW_RSP_OFF_REG 3u
#define GW_RSP_OFF_LEN 4u /* 2 bytes, LE */

/*
 * Largest payload either direction. 1024 bytes is 89 ms at 115200 baud and
 * 11 ms at 921600, and it is comfortably more than the biggest single region,
 * so a whole region is always one transaction.
 *
 * Both sides MUST validate LEN against this before allocating or copying. v1's
 * unbounded rxIndex against a 64-byte buffer was a live stack overflow that a
 * single noisy byte could trigger; that must not survive the rewrite.
 */
#define GW_MAX_PAYLOAD 1024u
#define GW_MAX_FRAME (GW_REQ_HEADER_LEN + GW_MAX_PAYLOAD + GW_CRC_LEN)

/* ==========================================================================
 * 3. Opcodes
 * ==========================================================================
 * Implemented in this first step: ECHO, SYNC, RD_META, RD_REGION, WR_REGION.
 * The rest are defined now so the numbering never has to move, and answered
 * with GW_ST_NOT_IMPL until their phase lands. The phase numbers refer to the
 * build order in docs/UNIVERSAL_PROCESS_IMAGE.html.
 */
#define GW_OP_RD_REGION 0x01u  /* region+offset+count -> raw bytes from the image */
#define GW_OP_WR_REGION 0x02u  /* host-owned regions only (e.g. Modbus TCP mirror) */
#define GW_OP_RD_META 0x03u    /* region descriptors + config identity            */
#define GW_OP_RD_DELTA 0x04u   /* P8: only slots changed since a watermark        */
#define GW_OP_WR_CHANNEL 0x05u /* P8: queue a field write, returns a token        */
#define GW_OP_RD_ACK 0x06u     /* P8: poll the result of a queued write           */
#define GW_OP_RD_EVENTS 0x07u  /* P8: pop from the event ring                     */
#define GW_OP_SYNC 0x08u       /* identity + uptime handshake, run at boot        */
#define GW_OP_CFG_BEGIN 0x10u  /* P5: config download, PC -> ESP32 -> STM32 flash */
#define GW_OP_CFG_CHUNK 0x11u
#define GW_OP_CFG_COMMIT 0x12u
#define GW_OP_CFG_ABORT 0x13u
#define GW_OP_ECHO 0x7Fu /* returns its payload verbatim - link bring-up only */

/* ==========================================================================
 * 4. Status codes
 * ==========================================================================
 * STATUS answers "what happened", not "did I understand the frame". That
 * distinction is the fix for the worst defect in v1, where a Modbus timeout was
 * reported to the cloud as OK together with stale bytes.
 *
 * Note what is NOT here: field-device health. A slave that stopped answering is
 * not a link error - RD_REGION still succeeds, and the per-slot quality byte
 * inside the payload says the value is bad. Link status is about the link.
 */
#define GW_ST_OK 0x00u
#define GW_ST_BAD_CRC 0x01u      /* our CRC check of the request failed           */
#define GW_ST_BAD_CMD 0x02u      /* unknown opcode                                */
#define GW_ST_BAD_REGION 0x03u   /* region id does not exist                      */
#define GW_ST_RANGE 0x04u        /* offset+count runs past the end of the region  */
#define GW_ST_BUSY 0x05u         /* transient; retry is expected to work          */
#define GW_ST_CFG_MISMATCH 0x06u /* caller's config CRC is not ours               */
#define GW_ST_NOT_WRITABLE 0x07u /* region or slot is not externally owned        */
#define GW_ST_QUEUE_FULL 0x08u   /* write queue full, nothing was accepted        */
#define GW_ST_FLASH_ERR 0x09u    /* config write failed                           */
#define GW_ST_NOT_IMPL 0x0Au     /* opcode reserved but not built yet             */
#define GW_ST_BAD_LEN 0x0Bu      /* LEN impossible for this opcode                */

/* ==========================================================================
 * 5. Regions
 * ==========================================================================
 * The image is one flat byte array cut into a region per data source. A region
 * id is stable forever; its size and position are NOT - the ESP32 learns those
 * from RD_META at boot and must never hardcode them. Resizing a region is then
 * a configuration change rather than a firmware change on two chips.
 */
#define GW_REGION_SYS 0x00u      /* uptime, identity, link and scan statistics    */
#define GW_REGION_LOCAL_IO 0x01u /* this module's own DI / DO / AI / AO           */
#define GW_REGION_MB_RTU 0x02u   /* RS-485 slaves - the only one populated today  */
#define GW_REGION_MB_TCP 0x03u   /* written by whoever runs the TCP master        */
#define GW_REGION_CAN 0x04u      /* decoded CAN signals, refreshed on receive     */
#define GW_REGION_SERIAL 0x05u   /* custom / ASCII serial devices                 */
#define GW_REGION_VIRTUAL 0x06u  /* computed tags: totalisers, derived alarms     */
#define GW_REGION_EVENTS 0x07u   /* ring buffer of timestamped transitions        */
#define GW_REGION_COUNT 8u
#define GW_REGION_ALL 0xFFu /* RD_META only: "describe every region"         */

/* Region descriptor flags (GwRegionDesc.flags). */
#define GW_REGF_ENABLED 0x01u  /* allocated and served                           */
#define GW_REGF_RAW 0x02u      /* opaque bytes, no slot structure (SYS, EVENTS)   */
#define GW_REGF_WRITABLE 0x04u /* WR_REGION is accepted here                      */
#define GW_REGF_EXTERNAL 0x08u /* filled by the ESP32, not by an STM32 driver;
                                * needs the staleness watchdog described below    */

/* ==========================================================================
 * 6. Quality
 * ==========================================================================
 * Quality travels with every value. Codes 0..3 deliberately match the existing
 * TagQuality enum in IoTHandler/include/process_image.h, so the ESP32's cloud
 * layer needed no change when v2 arrived.
 *
 * The rule the whole design exists to enforce: a value whose quality is not
 * GW_Q_GOOD must never be published as a plain number. Publish it with its
 * quality, or do not publish it. Silently shipping a stale reading as good is
 * worse than shipping nothing, because nothing downstream can detect it.
 */
#define GW_Q_UNKNOWN 0u    /* never read since boot                            */
#define GW_Q_GOOD 1u       /* fresh and trustworthy                            */
#define GW_Q_STALE 2u      /* last read succeeded, but longer ago than staleMs */
#define GW_Q_COMM_FAIL 3u  /* the device stopped answering                     */
#define GW_Q_EXCEPTION 4u  /* device answered with a protocol exception        */
#define GW_Q_CONFIG_ERR 5u /* the channel definition itself is wrong           */
#define GW_Q_OVERRANGE 6u  /* raw value outside the configured limits          */
#define GW_Q_DISABLED 7u   /* channel exists but is switched off               */

/* ==========================================================================
 * 7. Slot layout inside a structured region
 * ==========================================================================
 * A region holds `slots` channels as THREE PARALLEL BLOCKS, not as an array of
 * structs:
 *
 *   [ value[0..n-1] ][ stamp[0..n-1] ][ quality[0..n-1] ]
 *      4 bytes each     4 bytes each     1 byte each
 *
 * Parallel blocks because the consumer nearly always wants "all the values" or
 * "all the qualities", and a block is one contiguous read - one transaction
 * instead of n, and one memcpy on the STM32. An array of structs would make
 * every such read a strided gather.
 *
 * Fixed 4-byte value slots cover bool, u16, i16, u32, i32 and f32 with a single
 * arithmetic rule (offset = slot * 4). The wasted bytes are irrelevant on a
 * 192 KB part, and the win is that the ESP32 stays completely ignorant of data
 * types: it copies 4 bytes and looks up how to interpret them in its tag map.
 *
 * Blocks are ordered value, stamp, quality on purpose. Both 4-byte blocks come
 * first so that the stamp block starts at a 4-aligned offset for any slot
 * count. That matters because on Cortex-M4 an aligned 32-bit store is atomic,
 * which is what lets the scanner write while the link reads with no mutex
 * anywhere - a reader can never catch half a value. It can catch slot 5 being
 * a few milliseconds newer than slot 4, and that is exactly what a process
 * image is; the stamp block is how a consumer sees it.
 *
 * Stamps are milliseconds since STM32 boot. There is deliberately no RTC on the
 * industrial half: the ESP32 has NTP and converts to wall clock using the
 * uptime returned by SYNC. One less time source to keep correct in the field.
 */
#define GW_SLOT_VALUE_BYTES 4u
#define GW_SLOT_STAMP_BYTES 4u
#define GW_SLOT_QUAL_BYTES 1u
#define GW_SLOT_TOTAL_BYTES (GW_SLOT_VALUE_BYTES + GW_SLOT_STAMP_BYTES + GW_SLOT_QUAL_BYTES) /* 9 */

/* ==========================================================================
 * 8. Payload structures
 * ==========================================================================
 * All packed, all little endian, all fixed size. They are memcpy'd straight out
 * of the frame on both sides, so their layout is part of the wire contract:
 * adding a field to the middle of one of these is a breaking change.
 */

/** RD_REGION payload: how many bytes the caller wants from REG at OFF. */
typedef struct __attribute__((packed)) {
    uint16_t count;
} GwReadArgs;

/**
 * One entry of the RD_META reply. The ESP32 keeps the whole table and derives
 * every address it will ever use from it: value i lives at
 * valueOff + i * 4, its stamp at stampOff + i * 4, its quality at qualOff + i.
 */
typedef struct __attribute__((packed)) {
    uint8_t region;    /* GW_REGION_*                                       */
    uint8_t flags;     /* GW_REGF_*                                         */
    uint16_t slots;    /* structured regions only; 0 when GW_REGF_RAW       */
    uint16_t byteLen;  /* usable bytes in this region                       */
    uint16_t valueOff; /* all three offsets are relative to the region base  */
    uint16_t stampOff;
    uint16_t qualOff;
} GwRegionDesc; /* 12 bytes */

/**
 * RD_META reply header, followed by `regions` GwRegionDesc entries.
 *
 * configCrc32 is the interlock that prevents the worst failure this
 * architecture can produce. Once a PC tool can reassign slots, "slot 5 of
 * region 0x02" stops meaning the same thing forever. If the ESP32 publishes
 * using an older map than the STM32 is filling, every tag is mislabelled -
 * pressure reported as temperature - and nothing anywhere raises an error. So
 * the config CRC is stamped into both the STM32 blob and the ESP32 tag map, is
 * returned here and in the SYS region, and the ESP32 refuses to publish on a
 * mismatch. Cheap to add now; impossible to retrofit after the first field
 * incident. Until the PC tool exists the CRC is a build-time constant.
 */
typedef struct __attribute__((packed)) {
    uint8_t protoVersion;
    uint8_t regions;
    uint16_t imageBytes;
    uint32_t configCrc32;
    uint16_t configVersion;
    uint16_t reserved;
} GwMetaHeader; /* 12 bytes */

/** SYNC request payload: what the ESP32 tells the STM32 about itself. */
typedef struct __attribute__((packed)) {
    uint32_t espUptimeMs;
    uint32_t espEpochSec; /* 0 when NTP has not resolved yet */
} GwSyncReq; /* 8 bytes */

/** SYNC reply payload: identity and liveness of the industrial half. */
typedef struct __attribute__((packed)) {
    uint8_t protoVersion;
    uint8_t fwMajor;
    uint8_t fwMinor;
    uint8_t fwPatch;
    uint32_t stmUptimeMs;
    uint32_t configCrc32;
    uint16_t configVersion;
    uint16_t capabilities; /* GW_CAP_* */
} GwSyncRsp; /* 16 bytes */

/* What this STM32 build can actually do. The ESP32 reads these instead of
 * guessing from a firmware version number, so a field unit running older
 * firmware degrades instead of erroring. */
#define GW_CAP_RD_DELTA 0x0001u
#define GW_CAP_WR_CHANNEL 0x0002u
#define GW_CAP_EVENTS 0x0004u
#define GW_CAP_CFG_DOWNLOAD 0x0008u
#define GW_CAP_EVENT_PIN 0x0010u /* STM32 drives the STM_EVENT side-band GPIO */
#define GW_CAP_LOCAL_IO 0x0020u  /* LOCAL_IO region is driven by a real IO driver */

/*
 * How to interpret the 4 value bytes of a WR_CHANNEL. The image itself is
 * typeless - it stores four bytes and nothing else - so the encoding has to
 * travel with the write. Without it a float setpoint of 1.0 and an integer 1
 * are the same request, and only one of them is right.
 */
#define GW_ENC_U32 0u /* unsigned integer, also used for bool 0/1 */
#define GW_ENC_I32 1u /* signed integer, two's complement          */
#define GW_ENC_F32 2u /* IEEE-754 single, bit pattern not converted */

/**
 * WR_CHANNEL request payload.
 *
 * REG carries the region and OFF carries the slot index, exactly as they do for
 * RD_REGION, so the payload only has to say what to write. Nothing here
 * addresses a field device: the caller names a slot, and whichever driver owns
 * that slot decides what "write slot 2" means on its bus. That is the whole
 * reason a cloud setpoint goes through WR_CHANNEL instead of WR_REGION - one
 * writer per slot, forever, with the owner deciding.
 */
typedef struct __attribute__((packed)) {
    uint32_t value;
    uint8_t encoding; /* GW_ENC_*                        */
    uint8_t flags;    /* reserved for P8, send 0          */
    uint16_t reserved;
} GwWrChannelReq; /* 8 bytes */

/**
 * WR_CHANNEL reply payload.
 *
 * `token` is the handle a caller would later poll with RD_ACK once queued
 * writes to slow buses land in P8. A driver that applies the write inside the
 * request - local IO does, it is one GPIO store - returns token 0 and
 * `applied` 1, which tells the caller the outcome is already final and there is
 * nothing to poll. Returning the field now rather than adding it later keeps
 * the frame layout stable when the queued path arrives.
 */
typedef struct __attribute__((packed)) {
    uint32_t token;
    uint8_t applied; /* 1 = already executed, 0 = queued, poll RD_ACK */
    uint8_t reserved[3];
} GwWrChannelRsp; /* 8 bytes */

/* ==========================================================================
 * 9. SYS region layout
 * ==========================================================================
 * Byte offsets inside region 0x00. Read it like any other region; it is just
 * flagged GW_REGF_RAW because it is a struct rather than slots. Everything here
 * is diagnostics: publish it as device attributes and a whole class of "is the
 * gateway healthy" question answers itself from the cloud dashboard.
 */
#define GW_SYS_OFF_UPTIME_MS 0x00u     /* u32 */
#define GW_SYS_OFF_CONFIG_CRC 0x04u    /* u32 */
#define GW_SYS_OFF_CONFIG_VER 0x08u    /* u16 */
#define GW_SYS_OFF_PROTO_VER 0x0Au     /* u8  */
#define GW_SYS_OFF_FW_MAJOR 0x0Bu      /* u8  */
#define GW_SYS_OFF_FW_MINOR 0x0Cu      /* u8  */
#define GW_SYS_OFF_FW_PATCH 0x0Du      /* u8  */
#define GW_SYS_OFF_BOOT_COUNT 0x0Eu    /* u16, 0 until config storage exists  */
#define GW_SYS_OFF_LINK_RX 0x10u       /* u32 frames accepted                 */
#define GW_SYS_OFF_LINK_TX 0x14u       /* u32 frames sent                     */
#define GW_SYS_OFF_LINK_CRC_ERR 0x18u  /* u32                                 */
#define GW_SYS_OFF_LINK_FRM_ERR 0x1Cu  /* u32 bad length / idle timeout       */
#define GW_SYS_OFF_LINK_DROPPED 0x20u  /* u32 bytes discarded hunting for SOF */
#define GW_SYS_OFF_LOOP_COUNT 0x24u    /* u32 superloop passes                */
#define GW_SYS_OFF_SCAN_CYCLE_US 0x28u /* u32, 0 until the RTU scanner lands  */
#define GW_SYS_OFF_FAULT_BITS 0x2Cu    /* u32 GW_FAULT_*                      */
#define GW_SYS_OFF_IMAGE_SEQ 0x30u     /* u32, ++ on every image write pass   */
#define GW_SYS_SIZE 128u               /* the rest is reserved, reads as zero */

#define GW_FAULT_NONE 0x00000000u
#define GW_FAULT_LINK_OVERRUN 0x00000001u     /* UART overran; DMA was restarted */
#define GW_FAULT_SCAN_OVERRUN 0x00000002u     /* a bus cannot meet its scan rate */
#define GW_FAULT_CFG_INVALID 0x00000004u      /* running on defaults             */
#define GW_FAULT_EXT_REGION_STALE 0x00000008u /* an EXTERNAL region froze        */

/* ==========================================================================
 * 10. LOCAL_IO slot map
 * ==========================================================================
 * Region 0x01 is this module's own terminals. Unlike a field bus region, whose
 * slot assignment comes from configuration, these slots are wired to physical
 * pins on the STM32 and so are fixed here in the contract: if the ESP32 and the
 * STM32 disagreed about which slot is relay 1, a dashboard switch would close
 * the wrong contactor, and nothing in the system could detect it.
 *
 *   slot  0..3   DO   relay 1..4       PD0..PD3, one slot per relay, 0 or 1
 *   slot  8..11  DI   input 1..4       PD4..PD7, pulled up, 0 or 1
 *   slot 16      DO word - all four relays packed, bit 0 = relay 1
 *   slot 17      DI word - all four inputs packed, bit 0 = input 1
 *
 * The gap between 3 and 8, and again before 16, is deliberate: it leaves room
 * for a wider IO card without renumbering anything that already exists in a
 * deployed dashboard or tag map.
 *
 * The two packed word slots are redundant with the per-bit slots and exist
 * because they are cheap and they make a consumer's life much easier: one
 * value to read for a change-detect, one attribute to publish, and an
 * unambiguous single-transaction snapshot of all four bits taken at the same
 * instant. The per-bit slots stay because a dashboard widget binds to one
 * value, not to a bit of one.
 *
 * WRITING: only slots 0..3 and 16 accept WR_CHANNEL. The DI slots are outputs
 * of the driver; writing one would be writing to an input terminal.
 */
#define GW_LIO_DO_BASE 0u   /* first relay slot                         */
#define GW_LIO_DO_COUNT 4u  /* PD0..PD3                                 */
#define GW_LIO_DI_BASE 8u   /* first input slot                         */
#define GW_LIO_DI_COUNT 4u  /* PD4..PD7                                 */
#define GW_LIO_DO_WORD 16u  /* all relays as a bitmask, bit 0 = relay 1 */
#define GW_LIO_DI_WORD 17u  /* all inputs as a bitmask, bit 0 = input 1 */
#define GW_LIO_SLOT_MAX 18u /* slots this map occupies                  */

/* ==========================================================================
 * 11. Shared helpers
 * ==========================================================================
 * Header-only, no allocation, safe on both sides. The name lookups exist so
 * logs on the ESP32 and a future STM32 trace agree word for word - chasing a
 * bug across two consoles that call the same thing different names wastes more
 * time than these tables cost in flash.
 */

/** Reads a little-endian u16 without assuming the pointer is aligned. */
static inline uint16_t gwRd16(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/** Reads a little-endian u32 without assuming the pointer is aligned. */
static inline uint32_t gwRd32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void gwWr16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static inline void gwWr32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline const char* gwStatusName(uint8_t st) {
    switch (st) {
        case GW_ST_OK: return "OK";
        case GW_ST_BAD_CRC: return "BAD_CRC";
        case GW_ST_BAD_CMD: return "BAD_CMD";
        case GW_ST_BAD_REGION: return "BAD_REGION";
        case GW_ST_RANGE: return "RANGE";
        case GW_ST_BUSY: return "BUSY";
        case GW_ST_CFG_MISMATCH: return "CFG_MISMATCH";
        case GW_ST_NOT_WRITABLE: return "NOT_WRITABLE";
        case GW_ST_QUEUE_FULL: return "QUEUE_FULL";
        case GW_ST_FLASH_ERR: return "FLASH_ERR";
        case GW_ST_NOT_IMPL: return "NOT_IMPL";
        case GW_ST_BAD_LEN: return "BAD_LEN";
        default: return "?";
    }
}

static inline const char* gwRegionName(uint8_t region) {
    switch (region) {
        case GW_REGION_SYS: return "SYS";
        case GW_REGION_LOCAL_IO: return "LOCAL_IO";
        case GW_REGION_MB_RTU: return "MB_RTU";
        case GW_REGION_MB_TCP: return "MB_TCP";
        case GW_REGION_CAN: return "CAN";
        case GW_REGION_SERIAL: return "SERIAL";
        case GW_REGION_VIRTUAL: return "VIRTUAL";
        case GW_REGION_EVENTS: return "EVENTS";
        case GW_REGION_ALL: return "ALL";
        default: return "?";
    }
}

static inline const char* gwQualityName(uint8_t q) {
    switch (q) {
        case GW_Q_UNKNOWN: return "unknown";
        case GW_Q_GOOD: return "good";
        case GW_Q_STALE: return "stale";
        case GW_Q_COMM_FAIL: return "commFail";
        case GW_Q_EXCEPTION: return "exception";
        case GW_Q_CONFIG_ERR: return "configErr";
        case GW_Q_OVERRANGE: return "overrange";
        case GW_Q_DISABLED: return "disabled";
        default: return "?";
    }
}

static inline const char* gwOpName(uint8_t op) {
    switch (op) {
        case GW_OP_RD_REGION: return "RD_REGION";
        case GW_OP_WR_REGION: return "WR_REGION";
        case GW_OP_RD_META: return "RD_META";
        case GW_OP_RD_DELTA: return "RD_DELTA";
        case GW_OP_WR_CHANNEL: return "WR_CHANNEL";
        case GW_OP_RD_ACK: return "RD_ACK";
        case GW_OP_RD_EVENTS: return "RD_EVENTS";
        case GW_OP_SYNC: return "SYNC";
        case GW_OP_CFG_BEGIN: return "CFG_BEGIN";
        case GW_OP_CFG_CHUNK: return "CFG_CHUNK";
        case GW_OP_CFG_COMMIT: return "CFG_COMMIT";
        case GW_OP_CFG_ABORT: return "CFG_ABORT";
        case GW_OP_ECHO: return "ECHO";
        default: return "?";
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* GW_MODEL_H */
