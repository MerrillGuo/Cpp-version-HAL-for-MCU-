#ifndef OSCL_COMMON_H
#define OSCL_COMMON_H

#include <stdint.h>
#include <functional>

namespace oscl {
    // 基础系统功能
    void delay_ms(uint32_t ms);
    uint32_t get_tick_ms();

    // 内存管理接口
    void* malloc(size_t size);
    void free(void* ptr);
    
    // 临界区管理
    void enter_critical();
    void exit_critical();
    bool is_in_isr();  // 是否在中断上下文中
    
    // 同步原语句柄定义
    using SemaphoreHandle = void*;
    using MutexHandle = void*;
    
    // 等待时间常量
    static constexpr uint32_t NO_WAIT = 0;
    static constexpr uint32_t WAIT_FOREVER = UINT32_MAX;

    // 信号量操作接口
    SemaphoreHandle semaphore_create(uint32_t initial_count, uint32_t max_count = 1);
    void semaphore_delete(SemaphoreHandle handle);
    bool semaphore_take(SemaphoreHandle handle, uint32_t timeout_ms);
    bool semaphore_give(SemaphoreHandle handle);
    bool semaphore_give_from_isr(SemaphoreHandle handle);

    // 互斥锁操作接口
    MutexHandle mutex_create();
    void mutex_delete(MutexHandle handle);
    bool mutex_lock(MutexHandle handle, uint32_t timeout_ms);
    bool mutex_unlock(MutexHandle handle);
    bool mutex_try_lock(MutexHandle handle);  // 非阻塞尝试获取锁
    
    // 任务优先级定义
    enum class Priority : uint8_t {
        Lowest  = 0,
        Low     = 1,
        Normal  = 2,
        High    = 3,
        Highest = 4,
        Count   = 5
    };

    // 任务状态监控
    enum class TaskState : uint8_t {
        Running,    // 运行中
        Ready,      // 就绪
        Blocked,    // 阻塞
        Suspended,  // 挂起
        Deleted,    // 已删除
        Invalid     // 无效
    };

    struct TaskInfo {
        char name[16];           // 任务名称
        uint8_t priority;        // 任务优先级
        TaskState state;         // 任务状态
        uint32_t stack_hwm;      // 栈最大使用量
    };

    struct SystemInfo {
        uint32_t uptime_ms;      // 系统运行时间(ms)
        uint32_t free_heap;      // 空闲堆内存(bytes)
        uint32_t total_heap;     // 总堆内存(bytes)
        uint32_t task_count;     // 任务数量
    };

    // 任务管理接口
    using TaskHandle = void*;
    using RawTaskFunction = void(*)(void*);  // 原始函数指针，零开销
    using TaskFunction = std::function<void()>; // 灵活的函数包装器

    struct TaskConfig {
        const char* name;        // 任务名称
        uint32_t stack_size;     // 栈大小(words)
        Priority priority;       // 任务优先级
    };

    // 使用原始函数指针的接口 - 最高性能
    TaskHandle task_create_raw(RawTaskFunction func, const TaskConfig& config, void* param);

    // 使用std::function的接口 - 更灵活
    TaskHandle task_create(TaskFunction&& func, const TaskConfig& config);

    void task_delete(TaskHandle handle);
    void task_suspend(TaskHandle handle);
    void task_resume(TaskHandle handle);
    void task_set_priority(TaskHandle handle, Priority priority);
    Priority task_get_priority(TaskHandle handle);

    // 任务信息查询
    bool get_task_info(TaskInfo& info);                    // 获取当前任务信息
    bool get_task_info(const char* name, TaskInfo& info);  // 获取指定任务信息
    TaskHandle get_current_task();                         // 获取当前任务句柄
    const char* get_current_task_name();  // 获取当前任务名称
    
    // 系统管理接口
    SystemInfo get_system_info();
}

#endif // OSCL_COMMON_H
