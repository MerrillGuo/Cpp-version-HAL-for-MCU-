#include "oscl_common.h"
#include "FreeRTOS.h"
#include "semphr.h"

namespace oscl {

// 静态信号量控制块
struct StaticSemaphore {
    StaticSemaphore_t control_block;
    SemaphoreHandle_t handle;
    bool is_used;
};

// 静态分配信号量池
static constexpr size_t MAX_SEMAPHORES = 8;  // 根据实际需求调整
static StaticSemaphore semaphore_pool[MAX_SEMAPHORES];

// 查找空闲信号量槽位
static StaticSemaphore* find_free_semaphore() {
    for (auto& sem : semaphore_pool) {
        if (!sem.is_used) {
            return &sem;
        }
    }
    return nullptr;
}

// 验证信号量句柄有效性
static bool is_valid_semaphore(SemaphoreHandle handle) {
    auto* sem = static_cast<StaticSemaphore*>(handle);
    if (sem < &semaphore_pool[0] || sem >= &semaphore_pool[MAX_SEMAPHORES]) {
        return false;
    }
    return sem->is_used;
}

SemaphoreHandle semaphore_create(uint32_t initial_count, uint32_t max_count) {
    // 参数检查
    if (max_count == 0 || initial_count > max_count) {
        return nullptr;
    }

    // 查找空闲槽位
    StaticSemaphore* sem = find_free_semaphore();
    if (!sem) {
        return nullptr;
    }

    // 创建信号量
    if (max_count == 1) {
        // 二值信号量
        sem->handle = xSemaphoreCreateBinaryStatic(&sem->control_block);
        if (sem->handle && initial_count == 1) {
            xSemaphoreGive(sem->handle);
        }
    } else {
        // 计数信号量
        sem->handle = xSemaphoreCreateCountingStatic(max_count, initial_count, &sem->control_block);
    }

    if (!sem->handle) {
        return nullptr;
    }

    sem->is_used = true;
    return sem;
}

void semaphore_delete(SemaphoreHandle handle) {
    if (!is_valid_semaphore(handle)) {
        return;
    }

    auto* sem = static_cast<StaticSemaphore*>(handle);
    vSemaphoreDelete(sem->handle);
    sem->is_used = false;
}

bool semaphore_take(SemaphoreHandle handle, uint32_t timeout_ms) {
    if (!is_valid_semaphore(handle)) {
        return false;
    }

    auto* sem = static_cast<StaticSemaphore*>(handle);
    TickType_t ticks = (timeout_ms == WAIT_FOREVER) ? 
                        portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    return xSemaphoreTake(sem->handle, ticks) == pdTRUE;
}

bool semaphore_give(SemaphoreHandle handle) {
    if (!is_valid_semaphore(handle)) {
        return false;
    }

    if (is_in_isr()) {
        return semaphore_give_from_isr(handle);
    }

    auto* sem = static_cast<StaticSemaphore*>(handle);
    return xSemaphoreGive(sem->handle) == pdTRUE;
}

bool semaphore_give_from_isr(SemaphoreHandle handle) {
    if (!is_valid_semaphore(handle)) {
        return false;
    }

    auto* sem = static_cast<StaticSemaphore*>(handle);
    BaseType_t higher_priority_task_woken = pdFALSE;
    BaseType_t result = xSemaphoreGiveFromISR(sem->handle, &higher_priority_task_woken);
    
    portYIELD_FROM_ISR(higher_priority_task_woken);
    return result == pdTRUE;
}

} // namespace oscl 