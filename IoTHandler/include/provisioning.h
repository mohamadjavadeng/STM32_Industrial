/**
 * provisioning.h - the commissioning console on UART0.
 * ---------------------------------------------------------------------------
 *
 * A line protocol on the same USB serial port that carries the log, spoken by
 * tools/gw_config_tool.py. It exists so that a panel in a cabinet can be given
 * its WiFi credentials and its ThingsBoard token with a USB cable and no
 * toolchain - see device_config.h for why that matters.
 *
 * THE ONE DESIGN DECISION WORTH KNOWING
 *   Every reply line starts with '+'. Nothing else printed by this firmware
 *   does.
 *
 *   The console shares a port with a running logger, so the tool cannot simply
 *   read the next line after sending a command - it would get whatever the poll
 *   task happened to print. Rather than muting the log (which hides exactly the
 *   messages that explain a failed connection), replies carry a marker the log
 *   never emits, and the tool ignores everything else. The operator keeps a
 *   readable log, the tool gets an unambiguous channel, and neither has to know
 *   about the other.
 *
 * PROTOCOL
 *   Commands are case-insensitive, terminated by newline. Every command ends
 *   its reply with either `+OK` or `+ERR <reason>`, so the tool always knows
 *   where a response stopped without relying on a timeout.
 *
 *     GW?              identify - answers +GW name=.. fw=.. proto=.. mac=..
 *     GET              dumps every config field as +<key>=<value>
 *     SET <key> <val>  sets one field in RAM. The value is the rest of the
 *                      line, so a WiFi password may contain spaces.
 *     SAVE             commits RAM to NVS
 *     CLEAR            erases NVS - the next boot is unprovisioned
 *     STATUS           live state: wifi, mqtt, link, relays, inputs
 *     REBOOT           +OK, then restarts
 *
 *   Keys are exactly the ones device_config.cpp stores: wifi.ssid, wifi.pass,
 *   tb.host, tb.port, tb.token, tb.tls, dev.name, tb.telemetryMs.
 *
 * SECURITY, HONESTLY STATED
 *   Anyone with physical access to the USB port can read the token and write a
 *   new one. That is the same trust boundary as the JTAG header and the flash
 *   chip next to it, so the console does not pretend otherwise with a password
 *   prompt. The WiFi passphrase is the one thing never echoed back, because it
 *   is usually shared with the rest of the site and is not this device's
 *   secret to give away.
 */
#pragma once

#include <Arduino.h>

/** Emits one `+key=value` line. Exposed so a status hook can use it. */
void provEmit(const char* key, const char* value);

/**
 * Called when the tool asks for STATUS, to add lines this module cannot know
 * about - MQTT state, link statistics, relay positions.
 *
 * A callback rather than an include, so that the console has no dependency on
 * the cloud client or the link and still builds in an environment where
 * neither is compiled.
 */
typedef void (*ProvStatusHook)(void);

void provisioningBegin(ProvStatusHook statusHook = nullptr);

/** Consumes whatever has arrived on UART0. Call from the main loop; never blocks. */
void provisioningPoll();
