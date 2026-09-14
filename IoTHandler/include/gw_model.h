/**
 * gw_model.h (ESP32 shim)
 *
 * The protocol contract is shared with the STM32 and lives in exactly one file,
 * <repo>/shared/gw_model.h. This shim pulls it in with a relative path so no
 * extra include directory is needed in platformio.ini - the same trick the
 * STM32 side uses in ModbusRTU_AWS/Core/Inc/gw_model.h.
 *
 * Path: IoTHandler/include -> IoTHandler -> repository root.
 *
 * Do not copy the shared header here. Two copies of a binary protocol drift,
 * and when they do, frames still parse and values are silently wrong.
 */
#pragma once

#include "../../shared/gw_model.h"
