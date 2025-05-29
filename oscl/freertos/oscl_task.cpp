#include "oscl_common.h"
#include "FreeRTOS.h"
#include "task.h"

namespace oscl {

static inline UBaseType_t convert_priority(Priority prio) {
    // FreeRTOS优先级从低到高是数字增大，这里做个转换
    return configMAX_PRIORITIES - 1 - static_cast<UBaseType_t>(prio);
}

static inline Priority convert_priority(UBaseType_t prio) {
    UBaseType_t converted = configMAX_PRIORITIES - 1 - prio;
    if (converted >= static_cast<UBaseType_t>(Priority::Count)) {
        return Priority::Normal;
    }
    return static_cast<Priority>(converted);
}

TaskHandle task_create_raw(RawTaskFunction func, const TaskConfig& config, void* param) {
    TaskHandle_t handle = nullptr;
    
    BaseType_t ret = xTaskCreate(
        reinterpret_cast<TaskFunction_t>(func),
        config.name,
        config.stack_size,
        param,
        convert_priority(config.priority),
        &handle
    );
    
    return ret == pdPASS ? handle : nullptr;
}

// 包装器结构体，用于存储std::function
struct TaskWrapper {
    TaskFunction func;
    explicit TaskWrapper(TaskFunction&& f) : func(std::move(f)) {}
};

// 任务包装函数
static void task_function_wrapper(void* param) {
    auto* wrapper = static_cast<TaskWrapper*>(param);
    wrapper->func();  // 执行实际的任务函数
    delete wrapper;   // 清理包装器
    vTaskDelete(nullptr);  // 删除任务
}

TaskHandle task_create(TaskFunction&& func, const TaskConfig& config) {
    // 创建包装器
    auto* wrapper = new(std::nothrow) TaskWrapper(std::move(func));
    if (!wrapper) {
        return nullptr;
    }

    TaskHandle_t handle = nullptr;
    BaseType_t ret = xTaskCreate(
        task_function_wrapper,
        config.name,
        config.stack_size,
        wrapper,
        convert_priority(config.priority),
        &handle
    );
    
    if (ret != pdPASS) {
        delete wrapper;
        return nullptr;
    }
    
    return handle;
}

void task_delete(TaskHandle handle) {
    vTaskDelete(static_cast<TaskHandle_t>(handle));
}

void task_suspend(TaskHandle handle) {
    vTaskSuspend(static_cast<TaskHandle_t>(handle));
}

void task_resume(TaskHandle handle) {
    vTaskResume(static_cast<TaskHandle_t>(handle));
}

void task_set_priority(TaskHandle handle, Priority priority) {
    vTaskPrioritySet(static_cast<TaskHandle_t>(handle), 
                     convert_priority(priority));
}

Priority task_get_priority(TaskHandle handle) {
    return convert_priority(uxTaskPriorityGet(static_cast<TaskHandle_t>(handle)));
}

TaskHandle get_current_task() {
    return xTaskGetCurrentTaskHandle();
}

void enter_critical() {
    taskENTER_CRITICAL();
}

void exit_critical() {
    taskEXIT_CRITICAL();
}

bool is_in_isr() {
    return xPortIsInsideInterrupt() != 0;
}

const char* get_current_task_name() {
    const char* name = pcTaskGetName(nullptr);
    return name ? name : "unknown";
}

} // namespace oscl 