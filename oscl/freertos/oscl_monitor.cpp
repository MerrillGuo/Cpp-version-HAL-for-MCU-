#include "oscl_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

namespace oscl {

static TaskState convert_state(eTaskState state) {
    switch (state) {
        case eRunning:   return TaskState::Running;
        case eReady:     return TaskState::Ready;
        case eBlocked:   return TaskState::Blocked;
        case eSuspended: return TaskState::Suspended;
        case eDeleted:   return TaskState::Deleted;
        default:         return TaskState::Invalid;
    }
}

bool get_task_info(TaskInfo& info) {
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (task == nullptr) {
        return false;
    }

    // 获取任务名称
    const char* name = pcTaskGetName(task);
    if (name) {
        strncpy(info.name, name, sizeof(info.name) - 1);
        info.name[sizeof(info.name) - 1] = '\0';
    } else {
        info.name[0] = '\0';
    }

    // 获取任务状态
    info.state = convert_state(eTaskGetState(task));
    info.priority = uxTaskPriorityGet(task);
    info.stack_hwm = uxTaskGetStackHighWaterMark(task);

    return true;
}

bool get_task_info(const char* name, TaskInfo& info) {
    TaskHandle_t task = xTaskGetHandle(name);
    if (task == nullptr) {
        return false;
    }

    strncpy(info.name, name, sizeof(info.name) - 1);
    info.name[sizeof(info.name) - 1] = '\0';
    
    info.state = convert_state(eTaskGetState(task));
    info.priority = uxTaskPriorityGet(task);
    info.stack_hwm = uxTaskGetStackHighWaterMark(task);

    return true;
}

SystemInfo get_system_info() {
    SystemInfo info = {};
    
    info.uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    info.free_heap = xPortGetFreeHeapSize();
    info.total_heap = configTOTAL_HEAP_SIZE;
    info.task_count = uxTaskGetNumberOfTasks();
    
    return info;
}

} // namespace oscl 