#include "test_dma.h"

#include "board_config.h"
#include "oscl_common.h"
#include "unity_fixture.h"
#include "elog.h"

#include <array>
#include <span>
#include <string_view>
#include <cstring>

namespace {

// 使用DMA0通道0
using TestDma = Dma<0, 0>;

// 测试内存缓冲区大小
constexpr size_t kBufferSize = 64;

// 源数据缓冲区和目标数据缓冲区
alignas(4) static std::array<uint32_t, kBufferSize> source_buffer;
alignas(4) static std::array<uint32_t, kBufferSize> destination_buffer;

// 测试状态标志
volatile bool transfer_complete = false;

// DMA传输完成回调函数
void dma_transfer_complete_callback() {
    transfer_complete = true;
}

// 重置测试状态
void reset_test_state() {
    // 重置传输完成标志
    transfer_complete = false;
    
    // 初始化源缓冲区和目标缓冲区
    for (size_t i = 0; i < kBufferSize; ++i) {
        source_buffer[i] = 8*i;  // 填充源缓冲区
        destination_buffer[i] = 0;  // 清空目标缓冲区
    }
}

// 验证DMA传输结果
bool verify_transfer_result() {
    // 比较源缓冲区和目标缓冲区
    for (size_t i = 0; i < kBufferSize; ++i) {
        if (source_buffer[i] != destination_buffer[i]) {
            printf("Transfer verification failed at index %d: expected %lu, got %lu\n", 
                   i, source_buffer[i], destination_buffer[i]);
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_GROUP(TestDma);

TEST_SETUP(TestDma) {
    // 初始化测试状态
    reset_test_state();
    printf("\nSetting up DMA test\n");
}

TEST_TEAR_DOWN(TestDma) {
    // 停止DMA并清理
    TestDma::stop();
    TestDma::deinit();
    oscl::delay_ms(10);  // 等待硬件稳定
    printf("\nTearing down DMA test\n");
}

// 定义测试用例
#define TEST_CASE(name) TEST(TestDma, name)

TEST_CASE(MemoryToMemoryWithInterrupt) {
    printf("\nTesting memory to memory DMA transfer with interrupt\n");
    
    // 重置测试状态
    reset_test_state();
    
    // 配置DMA为内存到内存传输，带传输完成中断
    auto status = TestDma::init()
        .direction(hcl::DmaDirection::kMemToMem)  // 使用内存到内存方向
        .periph_width(hcl::DmaWidth::k32Bit)
        .memory_width(hcl::DmaWidth::k32Bit)
        .periph_inc(hcl::DmaAddressIncrement::kEnabled)  // 源地址递增
        .memory_inc(hcl::DmaAddressIncrement::kEnabled)  // 目标地址递增
        .commit();
    
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
    
    // 设置传输完成回调
    TestDma::set_full_transfer_complete_callback(dma_transfer_complete_callback);
    
    // 启动DMA传输，动态传入地址和传输数量
    status = TestDma::start(
        reinterpret_cast<uint32_t>(source_buffer.data()),
        reinterpret_cast<uint32_t>(destination_buffer.data()),
        kBufferSize
    );
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
    
    // 等待中断回调
    uint32_t timeout = 1000;  // 1秒超时
    uint32_t start = oscl::get_tick_ms();
    while (!transfer_complete) {
        if (oscl::get_tick_ms() - start > timeout) {
            TEST_FAIL_MESSAGE("DMA transfer with interrupt timeout");
            return;
        }
    }
    
    // 确认传输完成
    TEST_ASSERT_TRUE(transfer_complete);

    printf("DMA transfer complete\n");
    
    // 验证传输结果
    bool result = verify_transfer_result();
    TEST_ASSERT_TRUE(result);
    
    printf("Memory to memory DMA transfer with interrupt completed successfully\n");
    
    // 打印传输结果示例（前几个元素）
    printf("Source buffer (first 5 elements): ");
    for (size_t i = 0; i < 5 && i < kBufferSize; ++i) {
        printf("%lu ", source_buffer[i]);
    }
    printf("\n");
    oscl::delay_ms(100);
    printf("Destination buffer (first 10 elements): ");
    for (size_t i = 0;  i < kBufferSize; ++i) {
        printf("%lu ", destination_buffer[i]);
    }
    oscl::delay_ms(100);
    printf("\n");
}

TEST_GROUP_RUNNER(TestDma) {
    RUN_TEST_CASE(TestDma, MemoryToMemoryWithInterrupt);
}

void RunAllTestsForTestDma(void) {
    RUN_TEST_GROUP(TestDma);
}
