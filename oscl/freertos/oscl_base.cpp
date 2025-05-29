#include "oscl_common.h"
#include "FreeRTOS.h"
#include "task.h"

namespace oscl {

void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t get_tick_ms() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

} // namespace oscl 