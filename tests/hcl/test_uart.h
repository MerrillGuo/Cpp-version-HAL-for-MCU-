#ifndef TEST_UART_H
#define TEST_UART_H

#include "board_config.h"

// 定义测试用的串口类型
using TestUart = Uart<0>;  // UART1用于回环测试

namespace test {

// 辅助trait用于获取UART实例编号
template<typename T>
struct UartInstance;

template<size_t Instance>
struct UartInstance<hcl::UartBase<hcl::gd32f3::UartImpl<Instance>>> {
    static constexpr size_t value = Instance;
};

// 主模板声明
template<typename T>
struct UartTraits;

// TestUart的特化
template<>
struct UartTraits<TestUart> {
    static constexpr size_t kInstance = UartInstance<TestUart>::value;
    
    // 从board_config.h中获取配置，使用提取的实例编号
    using ConfigType = hcl::gd32f3::UartConfig<kInstance>;
    
    static constexpr auto kTxMode = ConfigType::kTxMode;
    static constexpr auto kRxMode = ConfigType::kRxMode;
    static constexpr auto kRxTrigger = ConfigType::kRxTrigger;
};

template<typename Uart>
struct UartTraits {
    static constexpr hcl::UartTrigger kRxTrigger = hcl::UartTrigger::kIdleDetect;  // Or whatever default value
    static constexpr hcl::UartMode kTxMode = hcl::UartMode::kInterrupt;
    static constexpr hcl::UartMode kRxMode = hcl::UartMode::kInterrupt;
};

} // namespace test

#ifdef __cplusplus
extern "C" {
#endif

// Unity Fixture 测试组运行器
void RunAllTestsForTestUart(void);

// 测试辅助函数
void uart_test_set_up(void);
void uart_test_tear_down(void);

// 回环测试 - 根据配置的模式执行
void test_uart_loopback(void);

// 错误处理测试
void test_uart_error_handling(void);

// 边界条件测试
void test_uart_edge_cases(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_UART_H 