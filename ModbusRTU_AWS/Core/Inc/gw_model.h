/**
 * gw_model.h (STM32 shim)
 *
 * The protocol contract is shared with the ESP32 and lives in exactly one file,
 * <repo>/shared/gw_model.h. This shim pulls it in with a relative path so that
 * neither STM32CubeIDE nor PlatformIO needs an extra include directory - which
 * matters here, because CubeMX rewrites project settings and any include path
 * added by hand is one more thing to lose on the next regeneration.
 *
 * Path: Core/Inc -> Core -> ModbusRTU_AWS -> repository root.
 *
 * Do not copy the shared header into the project. Two copies of a binary
 * protocol drift, and when they do, frames still parse and values are silently
 * wrong. Keep the indirection.
 */
#ifndef INC_GW_MODEL_H_
#define INC_GW_MODEL_H_

#include "../../../shared/gw_model.h"

#endif /* INC_GW_MODEL_H_ */
