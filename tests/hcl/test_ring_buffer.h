#ifndef TEST_RING_BUFFER_H
#define TEST_RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

// Unity Fixture 测试组运行器
void RunAllTestsForTestRingBuffer(void);

// 测试辅助函数
void ring_buffer_set_up(void);
void ring_buffer_tear_down(void);

// 测试用例函数
void test_buffer_initial_state(void);
void test_single_push_pop(void);
void test_buffer_full(void);
void test_bulk_push_pop(void);
void test_wrap_around(void);
void test_peek(void);
void test_clear(void);
void test_error_cases(void);
void test_continuous_operations(void);
void test_boundary_values(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_RING_BUFFER_H 