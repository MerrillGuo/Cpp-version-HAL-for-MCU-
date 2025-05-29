#include "test_adc.h"

#include "board_config.h"
#include "oscl_common.h"
#include "unity_fixture.h"


#include <array>
#include <span>
#include <string_view>


#define LOG_TAG "test_adc"
#define LOG_LEVEL ELOG_LVL_DEBUG
#include "elog.h"


namespace {

// 使用test命名空间中定义的TestAdc
using TestAdc = Adc<0>;
uint8_t count = 0;

// 测试状态标志
volatile bool adc_conversion_complete = false;

// ADC转换回调函数
void adc_callback() {
    adc_conversion_complete = true;
    count++;
}

void reset_flag() {
    adc_conversion_complete = false;
}

void print_rsq() {
    // 读取并打印ADC_RSQ0寄存器值
    uint32_t rsq0_value = ADC_RSQ0(ADC0);
    // printf("ADC_RSQ0 register value: 0x%08lu\n", rsq0_value);
    printf("Channel bits: RSQ12[%lu] RSQ13[%lu] RSQ14[%lu] RSQ15[%lu] LEN[%lu]\n",
           (rsq0_value >> 0) & 0x1F,
           (rsq0_value >> 5) & 0x1F,
           (rsq0_value >> 10) & 0x1F,
           (rsq0_value >> 15) & 0x1F,
           ((rsq0_value >> 20) & 0x0F)+1);
    oscl::delay_ms(1000);

    // 读取并打印ADC_RSQ1寄存器值
    uint32_t rsq1_value = ADC_RSQ1(ADC0);
    // printf("ADC_RSQ1 register value: 0x%08lu\n", rsq1_value);
    printf("Channel bits: RSQ6[%lu] RSQ7[%lu] RSQ8[%lu] RSQ9[%lu] RSQ10[%lu] RSQ11[%lu]\n",
           (rsq1_value >> 0) & 0x1F,
           (rsq1_value >> 5) & 0x1F,
           (rsq1_value >> 10) & 0x1F,
           (rsq1_value >> 15) & 0x1F,
           (rsq1_value >> 20) & 0x1F,
           (rsq1_value >> 25) & 0x1F);
    oscl::delay_ms(1000);

    // 读取并打印ADC_RSQ2寄存器值
    uint32_t rsq2_value = ADC_RSQ2(ADC0);
    // printf("ADC_RSQ2 register value: 0x%08lu\n", rsq2_value);
    printf("Channel bits: RSQ0[%lu] RSQ1[%lu] RSQ2[%lu] RSQ3[%lu] RSQ4[%lu] RSQ5[%lu]\n",
           (rsq2_value >> 0) & 0x1F,
           (rsq2_value >> 5) & 0x1F,
           (rsq2_value >> 10) & 0x1F,
           (rsq2_value >> 15) & 0x1F,
           (rsq2_value >> 20) & 0x1F,
           (rsq2_value >> 25) & 0x1F);
    oscl::delay_ms(1000);
}

}  // namespace

TEST_GROUP(TestAdc);

TEST_SETUP(TestAdc) {
    // 初始化ADC，获取状态并检查
    // auto status = TestAdc::init().commit();
    // TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
    // print_rsq();
    oscl::delay_ms(1);
    // printf("\nTesting init OK\n");
}

TEST_TEAR_DOWN(TestAdc) {
    TestAdc::stop();
    TestAdc::deinit();
    oscl::delay_ms(10);  // 等待硬件稳定
    printf("\nTesting deinit OK\n");
}

// 定义测试用例
#define TEST_CASE(name) TEST(TestAdc, name)

TEST_CASE(SingleChannelNoInterrupt) {
    // printf("\nTesting single channel conversion without interrupt\n");
    reset_flag();

    auto status = TestAdc::init()
        .conv_mode(hcl::AdcConvMode::kContinuous)
        .channel_count(1)                             // 设置通道数量
        .set_channel(0, hcl::AdcChannel::kChannel4)   // 配置通道4
        .commit();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
    // print_rsq();


    // 启动转换
    status = TestAdc::start();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
    
    // 轮询等待转换完成
    uint32_t timeout = 1000;  // 1秒超时
    
    for (uint8_t i = 0; i < 5; ++i) {
        uint32_t start = oscl::get_tick_ms();
        while (!TestAdc::is_conversion_complete()) {
            if (oscl::get_tick_ms() - start > timeout) {
                TEST_FAIL_MESSAGE("ADC conversion timeout");
                return;
            }
        }
        
        // 读取并验证结果
        uint16_t result = TestAdc::read();
        printf("ADC value: %u, Count: %u\n", result, i + 1);
        
        // 基本有效性检查
        TEST_ASSERT_LESS_OR_EQUAL(4095, result);  // 12位ADC最大值
        TEST_ASSERT_GREATER_OR_EQUAL(0, result);  // 最小值检查
        
        oscl::delay_ms(300);
    }
}

TEST_CASE(MultiChannelWithInterrupt) {
    printf("\nTesting multi-channel conversion with DMA interrupt\n");
    reset_flag();
    count = 0;  // 重置计数器

    // 配置ADC为多通道DMA模式，禁用中断
    auto status = TestAdc::init()
        .conv_mode(hcl::AdcConvMode::kContinuous)
        .dma_mode(hcl::AdcDmaMode::kDmaEnabled)      // 启用DMA
        .channel_count(4)                             // 设置通道数量
        .set_channel(0, hcl::AdcChannel::kChannel0)   // 配置通道6
        .set_channel(1, hcl::AdcChannel::kChannel1)   // 配置通道7
        .set_channel(2, hcl::AdcChannel::kChannel2)   // 配置通道8
        .set_channel(3, hcl::AdcChannel::kChannel3)   // 配置通道9
        .set_channel(4, hcl::AdcChannel::kChannel4)   // 配置通道10
        .set_channel(5, hcl::AdcChannel::kChannel5)   // 配置通道11
        .set_channel(6, hcl::AdcChannel::kChannel6)   // 配置通道12
        .set_channel(7, hcl::AdcChannel::kChannel7)   // 配置通道13
        .set_channel(8, hcl::AdcChannel::kChannel8)   // 配置通道14
        .set_channel(9, hcl::AdcChannel::kChannel9)   // 配置通道15
        .set_channel(10, hcl::AdcChannel::kChannel10)   // 配置通道16
        .set_channel(11, hcl::AdcChannel::kChannel11)   // 配置通道17
        .set_channel(12, hcl::AdcChannel::kChannel12)   // 配置通道18
        .set_channel(13, hcl::AdcChannel::kChannel13)   // 配置通道19
        .set_channel(14, hcl::AdcChannel::kChannel14)   // 配置通道20
        .set_channel(15, hcl::AdcChannel::kChannel15)   // 配置通道21
        .commit();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    // 测试别的ADC0记得关闭
    // TestAdc::set_adc_callback(adc_callback);

    // 启动转换
    status = TestAdc::start();
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

    print_rsq();
    oscl::delay_ms(1000);

    for (uint8_t i = 0; i < 250; ++i) {
        
        // 等待DMA传输完成
        uint32_t timeout = 1000;  // 1秒超时
        uint32_t start = oscl::get_tick_ms();
        while (!adc_conversion_complete) {  
            if (oscl::get_tick_ms() - start > timeout) {
                TEST_FAIL_MESSAGE("DMA transfer timeout");
                return;
            }
        }
        
        // 重置传输完成标志
        adc_conversion_complete = false;
        
        // 使用read(channel)函数读取并打印通道5、6、7的值
        uint16_t ch6_value = TestAdc::read(hcl::AdcChannel::kChannel0);
        uint16_t ch7_value = TestAdc::read(hcl::AdcChannel::kChannel1);
        uint16_t ch8_value = TestAdc::read(hcl::AdcChannel::kChannel2);
        uint16_t ch9_value = TestAdc::read(hcl::AdcChannel::kChannel3);

        printf("Using read(channel): CH6: %u CH7: %u CH8: %u CH9: %u\n", 
               ch6_value, ch7_value, ch8_value, ch9_value);
        oscl::delay_ms(300);
    }
}

TEST_GROUP_RUNNER(TestAdc) {
    // RUN_TEST_CASE(TestAdc, SingleChannelNoInterrupt);
    RUN_TEST_CASE(TestAdc, MultiChannelWithInterrupt);
}

void RunAllTestsForTestAdc(void) {
    RUN_TEST_GROUP(TestAdc);
} 