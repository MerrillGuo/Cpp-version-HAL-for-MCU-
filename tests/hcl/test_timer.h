#ifndef TESTS_HCL_TEST_TIMER_H_
#define TESTS_HCL_TEST_TIMER_H_

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Unity Fixture 测试组运行器
void RunAllTestsForTestTimer(void);

// 测试辅助函数
void timer_test_set_up(void);
void timer_test_tear_down(void);

// 测试用例函数
void test_timer_init(void);
void test_timer_counter(void);
void test_timer_interrupt(void);
void test_timer_pwm(void);

#ifdef __cplusplus
}
#endif

#endif  // TESTS_HCL_TEST_TIMER_H_ 