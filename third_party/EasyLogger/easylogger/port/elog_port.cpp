/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */
 
#include <elog.h>
#include "board_config.h"
#include "oscl_common.h"
#include <cstdio>
#include <cstring>
#include <span>

// 日志输出同步信号量
static oscl::TaskHandle elog_task = nullptr;
static oscl::SemaphoreHandle elog_lock = nullptr;      // 输出互斥锁
static oscl::SemaphoreHandle elog_async = nullptr;     // 异步通知信号量
static oscl::SemaphoreHandle elog_dma_lock = nullptr;  // DMA传输完成信号量

static void elog_async_thread(void* param);

// DMA传输完成回调
static void uart_tx_complete() {
    if (elog_dma_lock) {
        oscl::semaphore_give_from_isr(elog_dma_lock);
    }
}

#ifdef __cplusplus
extern "C" {
#endif

// 静态缓冲区用于时间戳格式化
static char time_buf[16] = {0};
static char info_buf[32] = {0};

void elog_port_deinit(void);

/**
 * EasyLogger port initialize
 *
 * @return result
 */
ElogErrCode elog_port_init(void) {
    ElogErrCode result = ELOG_NO_ERR;

    // 创建信号量
    elog_lock = oscl::semaphore_create(1);
    elog_async = oscl::semaphore_create(0);
    elog_dma_lock = oscl::semaphore_create(1);

    if (!elog_lock || !elog_async || !elog_dma_lock) {
        result = ELOG_ERR;
        elog_port_deinit();
        return result;
    }

    // 注册UART发送完成回调
    DebugUart::set_tx_callback(uart_tx_complete);

    // 创建异步输出线程
    oscl::TaskConfig config{
        .name = "elog_async",
        .stack_size = 128,  // 根据实际需求调整栈大小
        .priority = oscl::Priority::Low
    };
    
    elog_task = oscl::task_create_raw(elog_async_thread, config, nullptr);
    if (!elog_task) {
        result = ELOG_ERR;
        elog_port_deinit();
        return result;
    }

    return result;
}

/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void) {
    // 删除任务
    if (elog_task) {
        oscl::task_delete(elog_task);
        elog_task = nullptr;
    }

    // 删除信号量
    if (elog_lock) {
        oscl::semaphore_delete(elog_lock);
        elog_lock = nullptr;
    }
    if (elog_async) {
        oscl::semaphore_delete(elog_async);
        elog_async = nullptr;
    }
    if (elog_dma_lock) {
        oscl::semaphore_delete(elog_dma_lock);
        elog_dma_lock = nullptr;
    }
}

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char *log, size_t size) {
    // 使用Debug UART发送日志
    DebugUart::send(std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(log), size));
}

/**
 * output lock
 */
void elog_port_output_lock(void) {
    // 在中断上下文中
    if (oscl::is_in_isr()) {
        // 已经在中断中，无需额外的同步
        return;
    }
    
    // 在任务上下文中，获取信号量
    if (elog_lock) {
        oscl::semaphore_take(elog_lock, oscl::WAIT_FOREVER);
    }
}

/**
 * output unlock
 */
void elog_port_output_unlock(void) {
    // 在中断上下文中
    if (oscl::is_in_isr()) {
        // 已经在中断中，无需额外的同步
        return;
    }
    
    // 在任务上下文中，释放信号量
    if (elog_lock) {
        oscl::semaphore_give(elog_lock);
    }
}

/**
 * get current time interface
 *
 * @return current time
 */
const char *elog_port_get_time(void) {
    // 获取系统运行时间(ms)
    uint32_t time = oscl::get_tick_ms();
    
    // 格式化时间戳 [HH:MM:SS.mmm]
    uint32_t ms = time % 1000;
    time /= 1000;
    uint8_t sec = time % 60;
    time /= 60;
    uint8_t min = time % 60;
    time /= 60;
    uint8_t hour = time % 24;
    
    snprintf(time_buf, sizeof(time_buf), "[%02u:%02u:%02u.%03lu]", 
             hour, min, sec, ms);
             
    return time_buf;
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void) {
    return oscl::get_current_task_name();
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void) {
    // 获取当前任务名称
    const char* name = oscl::get_current_task_name();
    
    // 如果在中断中，返回 "IRQ"
    if (oscl::is_in_isr()) {
        return "IRQ";
    }
    
    // 格式化任务信息
    snprintf(info_buf, sizeof(info_buf), "%s", name);
    return info_buf;
}

void elog_async_output_notice(void) {
    // 通知异步输出线程有新日志需要处理
    if (elog_async) {
        oscl::semaphore_give(elog_async);
    }
}

#ifdef __cplusplus
}
#endif

// 异步输出线程入口函数
static void elog_async_thread(void* param) {
    (void)param;

    size_t get_log_size = 0;
#ifdef ELOG_ASYNC_LINE_OUTPUT
    static char poll_get_buf[ELOG_LINE_BUF_SIZE - 4];
#else
    static char poll_get_buf[ELOG_ASYNC_OUTPUT_BUF_SIZE - 4];
#endif
    while(true) {
        // 等待日志输出请求
        oscl::semaphore_take(elog_async, oscl::WAIT_FOREVER);
        
        // 轮询获取并输出日志
        while (true) {
#ifdef ELOG_ASYNC_LINE_OUTPUT
            get_log_size = elog_async_get_line_log(poll_get_buf, sizeof(poll_get_buf));
#else
            get_log_size = elog_async_get_log(poll_get_buf, sizeof(poll_get_buf));
#endif
           if (get_log_size) {
                elog_port_output(poll_get_buf, get_log_size);
                oscl::semaphore_take(elog_dma_lock, 100);
            } else {
                break;
            }
        }
    }
}