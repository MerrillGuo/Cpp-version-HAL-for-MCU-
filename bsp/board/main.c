#include "gd32f30x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "elog.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Private function prototypes */
static void SystemClock_Config(void);

void MainTask(void *argument);

/**
 * @brief  Main program
 * @retval int
 */
int main(void) {
    /* Power-on delay */
    for (volatile uint32_t i = 0; i < 100000; i++) {
        __NOP();
    }

    /* System initialization */
    SystemClock_Config();

    // 设置优先级分组，FreeRTOS 要求使用4位抢占优先级，0位子优先级
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    // 初始化 EasyLogger
    elog_init();

    // 设置格式
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME|ELOG_FMT_T_INFO);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_LVL|ELOG_FMT_TAG);

    elog_start();
    
    // /* Create the main task */
    xTaskCreate(MainTask, "Main", 512, NULL, (configMAX_PRIORITIES - 1), NULL);

    // /* Start scheduler */
    vTaskStartScheduler();
    
    /* We should never get here as control is now taken by the scheduler */
    for(;;);
    return 0;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
static void SystemClock_Config(void) {
    SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);
    NVIC_SetPriority(SysTick_IRQn, 13);
}


/**
  * @brief  Function implementing the init task.
  * @param  argument: Not used
  * @retval None
  */
// __attribute__((weak)) void MainTask(void *argument)
// {
//     (void)argument;  /* Prevent unused parameter warning */
//     for(;;)
//     {
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

#ifdef __cplusplus
}
#endif
