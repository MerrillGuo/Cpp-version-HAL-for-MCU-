#include "oscl_common.h"

extern "C" {
// Forward declaration of FreeRTOS function
uint32_t xTaskGetTickCount(void);
}

namespace oscl {

// Internal implementation of get_tick_ms for tests
// This provides a real-time base that we can offset for tests
uint32_t _real_get_tick_ms() {
    // Use FreeRTOS tick count directly
    // FreeRTOS ticks are typically in milliseconds, which is what we need
    return xTaskGetTickCount();
}

} // namespace oscl
