#include "test_timer.h"

#include "board_config.h"
#include "oscl_common.h"
#include "unity_fixture.h"

#define LOG_TAG "test_timer"
#define LOG_LEVEL ELOG_LVL_DEBUG
#include "elog.h"

// 测试用定时器实例
using TestTimer = Timer<2>;  // 使用TIMER2作为测试实例
static volatile bool timer_callback_called = false;
int overflow_count = 0;

TEST_GROUP(TestTimer);

TEST_SETUP(TestTimer) {
    // 确保每次测试前都重置定时器
    TestTimer::deinit();
    timer_callback_called = false;
}

TEST_TEAR_DOWN(TestTimer) {
    TestTimer::deinit();
}

// 定义所有测试用例
#define TEST_CASE(name) TEST(TestTimer, name)

// 定时器中断回调
static void timer_update_callback() {
    timer_callback_called = true;
    overflow_count++;
}

TEST_CASE(Counter) {
    hcl::Status status = TestTimer::init()
        .prescaler(7199)
        .period(9999)
        .commit();
    
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    // 启动定时器
    status = TestTimer::start();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    // 等待一小段时间，计数器应该有变化
    oscl::delay_ms(1);
    uint32_t counter = TestTimer::get_counter();
    TEST_ASSERT_NOT_EQUAL(0, counter);

    // 停止定时器
    status = TestTimer::stop();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
}

TEST_CASE(Interrupt) {
    // 先设置回调，确保在定时器初始化前就准备好
    timer_callback_called = false;
    TestTimer::set_callback(timer_update_callback);

    // 初始化定时器
    auto initializer = TestTimer::init();
    initializer.prescaler(7199)    // 72MHz / 7200 = 10KHz
              .period(9999)        // 10KHz / 10000 = 1Hz
              .mode(hcl::TimerMode::kNormal);
    
    auto status = initializer.commit();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    printf("Timer interrupt test starting...\n");
    
    // 启动定时器
    status = TestTimer::start();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    // 等待中断发生
    oscl::delay_ms(2000);
        
    TEST_ASSERT_TRUE_MESSAGE(timer_callback_called, "Timer interrupt not detected");

    TestTimer::stop();
}

TEST_CASE(PWM) {
    //手动示波器测量
    hcl::Status status = TestTimer::init()
        .prescaler(7199)
        .period(9999)
        .commit();
    
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    // 配置PWM通道0
    status = TestTimer::template enable_pwm<3>();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    // 设置50%占空比
    status = TestTimer::template set_pwm_duty<3>(5000);
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    // 启动定时器
    status = TestTimer::start();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    // 运行一段时间
    oscl::delay_ms(100000);

    // 改变占空比到25%
    status = TestTimer::template set_pwm_duty<3>(2500);
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    oscl::delay_ms(100);

    // 禁用PWM
    status = TestTimer::template disable_pwm<3>();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
}

TEST_CASE(FrequencyAccuracy) {
    printf("\nTesting timer frequency accuracy...\n");
    
    // 测试不同频率
    const std::array<uint32_t, 9> test_freqs = {
        1,      // 1Hz
        10,      // 10Hz
        50,      // 50Hz
        100,     // 100Hz
        200,     // 200Hz
        500,     // 500Hz
        1000,    // 1000Hz
        2000,    // 2000Hz
        5000     // 5000Hz
    };
    
    for(auto freq : test_freqs) {
        printf("Testing frequency: %lu Hz\n", freq);
        
        // 计算分频值和重载值
        auto [psc, arr] = TestTimer::calc_frequency_params(freq);
        printf("Prescaler: %d, Period: %ld\n", psc, arr);
        
        TestTimer::set_callback(timer_update_callback);
            
        auto status = TestTimer::init().prescaler(psc)
                  .period(arr)
                  .mode(hcl::TimerMode::kNormal)
                  .commit();
        TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

        // 开始测量
        uint32_t start_time = oscl::get_tick_ms();

        status = TestTimer::start();
        TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
        
        // 每100ms打印一次计数器值，直到1200ms
        const uint32_t monitor_interval = 100;  // 每100ms打印一次
        uint32_t elapsed = 0;
        while(elapsed < 1200) {
            oscl::delay_ms(monitor_interval);
            elapsed = oscl::get_tick_ms() - start_time;
            
            uint32_t counter = TestTimer::get_counter();
            printf("Time: %lu ms, Counter: %lu, Overflows: %d\n", 
                   elapsed, counter, overflow_count);
        }
        
        TestTimer::stop();

        // 获取最终计数值
        uint32_t end_time = oscl::get_tick_ms();
        uint32_t final_counter = TestTimer::get_counter();
        
        // 计算总计数值 = 溢出次数 * (arr + 1) + 当前计数值
        uint64_t total_counts = overflow_count * (arr + 1) + final_counter;    
        float elapsed_sec = (end_time - start_time)/1000.0f;
        
        // 计算实际频率
        uint32_t actual_freq = (total_counts/(arr + 1))/(elapsed_sec) ;
        
        printf("Expected: %lu Hz, Actual: %lu Hz (overflow count: %d)\n", 
               freq, actual_freq, overflow_count);
        
        overflow_count = 0;
    }
}

TEST_GROUP_RUNNER(TestTimer) {
    // RUN_TEST_CASE(TestTimer, Counter);
    // RUN_TEST_CASE(TestTimer, Interrupt);
    RUN_TEST_CASE(TestTimer, PWM);
    // RUN_TEST_CASE(TestTimer, FrequencyAccuracy);
}

void RunAllTestsForTestTimer(void) {
    RUN_TEST_GROUP(TestTimer);
}
