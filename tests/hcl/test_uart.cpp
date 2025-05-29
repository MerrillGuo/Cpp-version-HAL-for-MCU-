#include "test_uart.h"

#include "gpio.h"
#include "board_config.h"
#include "oscl_common.h"
#include "unity_fixture.h"

#include <array>
#include <span>
#include <string_view>

namespace {
// 测试用的回调函数
bool rx_callback_called = false;
bool tx_callback_called = false;
bool error_callback_called = false;
size_t received_length = 0;
hcl::UartError last_error = hcl::UartError::kNone;


void rx_callback(size_t len) {
  rx_callback_called = true;
  received_length = len;
}

void tx_callback() {
  tx_callback_called = true;
}

// 测试数据
constexpr std::array<uint8_t, 16> kTestPattern = {
    0x55, 0xAA, 0x00, 0xFF, 0x01, 0x02, 0x04, 0x08,
    0x10, 0x20, 0x40, 0x80, 0xF0, 0x0F, 0xC3, 0x3C
};
// constexpr std::array<uint8_t, 1> kTestPattern = {
//     0x55
// };
std::array<uint8_t, 64> rx_buffer{};

// 添加大数据量测试用的缓冲区
constexpr size_t kLargeBufferSize = 1024;  // 1KB 数据
// std::array<uint8_t, kLargeBufferSize> large_tx_buffer{};
// std::array<uint8_t, kLargeBufferSize> large_rx_buffer{};

// 辅助函数
void reset_flags() {
  rx_callback_called = false;
  tx_callback_called = false;
  error_callback_called = false;
  received_length = 0;
  last_error = hcl::UartError::kNone;
}

// 等待超时辅助函数
bool wait_for_condition(bool& condition, uint32_t timeout_ms = 1000) {
  uint32_t start = oscl::get_tick_ms();
  while (!condition) {
    if (oscl::get_tick_ms() - start > timeout_ms) {
      printf("Timeout waiting for condition after %lu ms\n", timeout_ms);
      return false;
    }
    oscl::delay_ms(1);
  }
  return true;
}

// 回环测试辅助函数
void do_loopback_test(std::span<const uint8_t> test_data = std::span{kTestPattern}) {
  reset_flags();
  rx_buffer.fill(0);

  if constexpr (test::UartTraits<TestUart>::kTxMode == hcl::UartMode::kBlocking && 
                test::UartTraits<TestUart>::kRxMode == hcl::UartMode::kBlocking) {
    printf("Uart test in blocking mode\n");
    
    // 阻塞模式回环测试
    hcl::Status send_status = TestUart::send(test_data);
    TEST_ASSERT_EQUAL(hcl::Status::kOk, send_status);

    // 等待数据发送完成
    oscl::delay_ms(50);

    // 读取回环数据
    std::span<uint8_t> rx_data{rx_buffer.data(), test_data.size()};
    size_t received = TestUart::read(rx_data);
    
    TEST_ASSERT_EQUAL(test_data.size(), received);
    TEST_ASSERT_EQUAL_MEMORY(test_data.data(), rx_buffer.data(), 
                            test_data.size());
  }
  else if constexpr (test::UartTraits<TestUart>::kTxMode == hcl::UartMode::kInterrupt && 
                     test::UartTraits<TestUart>::kRxMode == hcl::UartMode::kInterrupt) {
    TestUart::set_tx_callback(tx_callback);
    TestUart::set_rx_callback(rx_callback);

    if constexpr (test::UartTraits<TestUart>::kRxTrigger == hcl::UartTrigger::kIdleDetect) {
      printf("Uart test in interrupt mode with idle detect trigger\n");
      // 空闲检测模式测试
      hcl::Status recv_status = TestUart::start_receive();
      TEST_ASSERT_EQUAL(hcl::Status::kOk, recv_status);

      // 一次性发送所有数据
      hcl::Status send_status = TestUart::send(test_data);
      TEST_ASSERT_EQUAL(hcl::Status::kOk, send_status);
      
      // 等待发送完成
      TEST_ASSERT_TRUE(wait_for_condition(tx_callback_called));
    } else {
      printf("Uart test in interrupt mode with normal trigger\n");
      // 普通中断模式测试
      hcl::Status recv_status = TestUart::start_receive(test_data.size());
      TEST_ASSERT_EQUAL(hcl::Status::kOk, recv_status);

      // 发送测试数据
      hcl::Status send_status = TestUart::send(test_data);
      TEST_ASSERT_EQUAL(hcl::Status::kOk, send_status);

      // 等待发送完成
      TEST_ASSERT_TRUE(wait_for_condition(tx_callback_called));
    }

    // 等待接收完成
    TEST_ASSERT_TRUE(wait_for_condition(rx_callback_called));
    TEST_ASSERT_EQUAL(test_data.size(), received_length);

    // 验证接收数据
    std::span<uint8_t> rx_data{rx_buffer.data(), test_data.size()};
    size_t received = TestUart::read(rx_data);
    TEST_ASSERT_EQUAL(test_data.size(), received);
    TEST_ASSERT_EQUAL_MEMORY(test_data.data(), rx_buffer.data(),
                            test_data.size());

    TestUart::stop_receive();
  }
}

// // 生成测试数据的辅助函数
// void generate_test_pattern(std::span<uint8_t> buffer) {
//     for (size_t i = 0; i < buffer.size(); ++i) {
//         buffer[i] = static_cast<uint8_t>(i & 0xFF);
//     }
// }

// // 计算传输速率的辅助函数
// float calculate_transfer_rate(size_t bytes, uint32_t time_ms) {
//     return (bytes * 1000.0f) / (time_ms * 1024.0f);  // 返回 KB/s
// }
}  // namespace

TEST_GROUP(TestUart);

TEST_SETUP(TestUart) {
  reset_flags();  // 重置回调标志
  rx_buffer.fill(0);  // 清空接收缓冲区
  
  // 初始化UART - 使用标准配置
  TestUart::init()
      .baudrate(115200)
      .parity(hcl::UartParity::kNone)
      .stop_bits(hcl::UartStopBits::kBits1)
      .length(hcl::UartWordLength::kBits8);
}

TEST_TEAR_DOWN(TestUart) {
  TestUart::stop_receive();  // 停止接收
  TestUart::deinit();
  oscl::delay_ms(10);  // 等待硬件稳定
}

// 定义所有测试用例
#define TEST_CASE(name) TEST(TestUart, name)

TEST_CASE(Loopback) {
  do_loopback_test();
}

// TEST_CASE(EdgeCases) {
//   // 测试极限波特率
//   printf("Test baudrate: 1200\n");
  
//   // 2. 初始化并测试低波特率
//   TestUart::init().baudrate(1200);
//   do_loopback_test();

//   printf("Test baudrate: 921600\n");
  
//   // 4. 重新初始化高波特率
//   TestUart::init().baudrate(921600);
//   do_loopback_test();
//   oscl::delay_ms(100);

//   // 5. 数据长度测试也需要类似处理
//   for (size_t len = 1; len <= kTestPattern.size(); len++) {

//     printf("Test data length: %d\n", len);
//     do_loopback_test(std::span{kTestPattern.data(), len});
    
//     // 增加测试间隔，确保硬件完全复位
//     oscl::delay_ms(100);
//   }
// }

// TEST_CASE(BaudRateAccuracy) {
//     // 测试不同波特率的精确性
//     const std::array<uint32_t, 17> test_baudrates = {
//         1200, 2400,  4800, 9600, 14400, 19200, 38400, 56000, 57600, 
//         115200, 128000, 230400, 256000, 460800, 512000, 750000, 921600
//     };
    
//     for(auto baud : test_baudrates) {
//         printf("Testing baudrate: %lu\n", baud);
//         TestUart::init().baudrate(baud);
        
//         // 发送精确定时的数据模式
//         do_loopback_test();
//         oscl::delay_ms(100);
//     }
// }

// TEST_CASE(NoiseResistance) {
//     // 测试在不同数据模式下的抗干扰能力
//     const std::array<uint8_t, 4> test_patterns[] = {
//         {0x00, 0x00, 0x00, 0x00},  // 全0
//         {0xFF, 0xFF, 0xFF, 0xFF},  // 全1
//         {0x55, 0x55, 0x55, 0x55},  // 交替0/1
//         {0xAA, 0xAA, 0xAA, 0xAA}   // 交替1/0
//     };
    
//     for(const auto& pattern : test_patterns) {
//         do_loopback_test(std::span{pattern});
//         oscl::delay_ms(50);
//     }
// }

// TEST_CASE(IdleLineDetection) {
//     printf("\nTesting UART idle line detection...\n");
    
//     if constexpr (test::UartTraits<TestUart>::kRxTrigger == hcl::UartTrigger::kIdleDetect) {
//         bool idle_detected = false;
//         size_t received_length = 0;
        
//         TestUart::set_rx_callback([&](size_t len) {
//             idle_detected = true;
//             received_length = len;
//         });
        
//         // 启动接收
//         auto status = TestUart::start_receive();
//         TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
        
//         // 发送数据包，之后产生空闲间隔
//         const std::array<uint8_t, 16> test_data = {
//             0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA,
//             0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA
//         };
        
//         TestUart::send(std::span{test_data});
//         oscl::delay_ms(100);  // 产生空闲间隔
        
//         TEST_ASSERT_TRUE(idle_detected);
        
//         TestUart::stop_receive();
//     }
// }

// TEST_CASE(LargeDataTransfer) {
//     printf("\nTesting large data transfer...\n");
    
//     // 生成测试数据
//     generate_test_pattern(large_tx_buffer);
//     large_rx_buffer.fill(0);
    
//     // 记录开始时间
//     uint32_t start_time = oscl::get_tick_ms();
//     bool transfer_complete = false;
//     size_t total_received = 0;
    
//     if constexpr (test::UartTraits<TestUart>::kTxMode == hcl::UartMode::kBlocking && 
//                  test::UartTraits<TestUart>::kRxMode == hcl::UartMode::kBlocking) {
//         printf("Large data transfer test in blocking mode\n");
        
//         // 分块发送和接收，避免缓冲区溢出
//         constexpr size_t chunk_size = 64;  // 每次传输64字节
//         for (size_t offset = 0; offset < kLargeBufferSize; offset += chunk_size) {
//             size_t current_chunk = std::min(chunk_size, kLargeBufferSize - offset);
            
//             // 发送数据块
//             std::span<const uint8_t> tx_chunk{large_tx_buffer.data() + offset, current_chunk};
//             hcl::Status send_status = TestUart::send(tx_chunk);
//             TEST_ASSERT_EQUAL(hcl::Status::kOk, send_status);
            
//             // 等待数据发送完成
//             oscl::delay_ms(5);
            
//             // 接收数据块
//             std::span<uint8_t> rx_chunk{large_rx_buffer.data() + offset, current_chunk};
//             size_t received = TestUart::read(rx_chunk);
//             total_received += received;
            
//             // 打印进度
//             if (offset % 256 == 0) {
//                 printf("Transferred: %zu bytes\n", offset + received);
//             }
//         }
        
//         transfer_complete = (total_received == kLargeBufferSize);
//     } 
//     else if constexpr (test::UartTraits<TestUart>::kTxMode == hcl::UartMode::kInterrupt && 
//                        test::UartTraits<TestUart>::kRxMode == hcl::UartMode::kInterrupt) {
//         printf("Large data transfer test in interrupt mode\n");
        
//         TestUart::set_tx_callback(tx_callback);
//         TestUart::set_rx_callback(rx_callback);
        
//         // 启动接收
//         if constexpr (test::UartTraits<TestUart>::kRxTrigger == hcl::UartTrigger::kIdleDetect) {
//             hcl::Status recv_status = TestUart::start_receive();
//             TEST_ASSERT_EQUAL(hcl::Status::kOk, recv_status);
//         } else {
//             hcl::Status recv_status = TestUart::start_receive(kLargeBufferSize);
//             TEST_ASSERT_EQUAL(hcl::Status::kOk, recv_status);
//         }
        
//         // 发送全部数据
//         hcl::Status send_status = TestUart::send(std::span{large_tx_buffer});
//         TEST_ASSERT_EQUAL(hcl::Status::kOk, send_status);
        
//         // 等待发送和接收完成
//         TEST_ASSERT_TRUE(wait_for_condition(tx_callback_called, 5000));
//         TEST_ASSERT_TRUE(wait_for_condition(rx_callback_called, 5000));
        
//         // 读取接收到的数据
//         std::span<uint8_t> rx_span{large_rx_buffer};
//         total_received = TestUart::read(rx_span);
//         transfer_complete = (total_received == kLargeBufferSize);
//     }
    
//     // 计算传输时间和速率
//     uint32_t elapsed_time = oscl::get_tick_ms() - start_time;
//     float transfer_rate = calculate_transfer_rate(total_received, elapsed_time);
    
//     // 输出测试结果
//     printf("Transfer complete: %s\n", transfer_complete ? "Yes" : "No");
//     printf("Total bytes transferred: %zu\n", total_received);
//     printf("Time elapsed: %lu ms\n", elapsed_time);
//     printf("Transfer rate: %.2f KB/s\n", transfer_rate);
    
//     // 验证数据完整性
//     TEST_ASSERT_EQUAL(kLargeBufferSize, total_received);
//     TEST_ASSERT_EQUAL_MEMORY(large_tx_buffer.data(), large_rx_buffer.data(), 
//                             kLargeBufferSize);
// }

// TEST_CASE(ParityModeTest) {
//     printf("\nTesting different parity modes...\n");
    
//     // 定义测试用的校验位模式
//     const std::array<std::pair<hcl::UartParity, const char*>, 3> parity_modes = {{
//         {hcl::UartParity::kNone, "None"},
//         {hcl::UartParity::kEven, "Even"},
//         {hcl::UartParity::kOdd,  "Odd"}
//     }};
    
//     // 测试数据：包含不同的位模式
//     const std::array<uint8_t, 4> test_data = {0x55, 0xAA, 0x33, 0xCC};
    
//     // 遍历每种校验位模式
//     for(const auto& [parity, name] : parity_modes) {
//         printf("Testing parity mode: %s\n", name);
        
//         auto initializer = TestUart::init();
//         initializer.baudrate(115200)
//                   .parity(parity)
//                   .stop_bits(hcl::UartStopBits::kBits1)
//                   .length(hcl::UartWordLength::kBits8);
        
//         do_loopback_test(std::span{test_data});
//     }
// }

// TEST_CASE(StopBitsTest) {
//     printf("\nTesting different stop bits modes...\n");
    
//     // 定义测试用的停止位模式
//     const std::array<std::pair<hcl::UartStopBits, const char*>, 2> stop_bits_modes = {{
//         {hcl::UartStopBits::kBits1, "1 bit"},
//         {hcl::UartStopBits::kBits2, "2 bits"}
//     }};
    
//     // 测试数据：包含不同的位模式
//     const std::array<uint8_t, 4> test_data = {0x55, 0xAA, 0x33, 0xCC};
    
//     // 遍历每种停止位模式
//     for(const auto& [stop_bits, name] : stop_bits_modes) {
//         printf("Testing stop bits mode: %s\n", name);
        
//         auto initializer = TestUart::init();
//         initializer.baudrate(115200)
//                   .parity(hcl::UartParity::kNone)
//                   .stop_bits(stop_bits)
//                   .length(hcl::UartWordLength::kBits8);
        
//         do_loopback_test(std::span{test_data});
        
//         oscl::delay_ms(100);  // 不同测试之间的间隔
//     }
// }

// TEST_CASE(DmaTxModeTest) {
//     printf("\nTesting DMA Tx mode...\n");
    
//     // 初始化测试数据
//     const uint8_t test_data1[] = "UART DMA TX TEST";
//     const uint8_t test_data2[] = "ABCDBNILHUIOLUIO123454";
//     const size_t data_len1 = sizeof(test_data1) - 1; // 不包括结尾的\0
//     const size_t data_len2 = sizeof(test_data2) - 1; // 不包括结尾的\0
//     tx_callback_called = false;
    
//     // 设置发送完成回调
//     TestUart::set_tx_callback(tx_callback);
        
//     // 清除缓冲区和状态
//     TestUart::clear_rx();
    
//     // 使用DMA发送数据
//     printf("Sending data via DMA: %s\n", test_data1);
//     auto status = TestUart::send(std::span{test_data1, data_len1});
//     TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
    
//     // 等待发送完成
//     uint32_t timeout = 1000; // 1秒超时
//     uint32_t start = oscl::get_tick_ms();
//     while (!tx_callback_called) {
//         if (oscl::get_tick_ms() - start > timeout) {
//             TEST_FAIL_MESSAGE("DMA transfer timeout");
//             break;
//         }
//         oscl::delay_ms(1);
//     }
    
//     // 验证发送已完成
//     TEST_ASSERT_TRUE(tx_callback_called);
//     printf("DMA Tx test completed successfully\n");
    
//     oscl::delay_ms(1000);

//     // 重置状态准备第二次发送
//     tx_callback_called = false;
//     TestUart::clear_rx();
//     printf("Sending data via DMA: %s\n", test_data2);
//     status = TestUart::send(std::span{test_data2, data_len2});
//     TEST_ASSERT_EQUAL(hcl::Status::kOk, status);

//     // 等待发送完成
//     while (!tx_callback_called) {
//         if (oscl::get_tick_ms() - start > timeout) {
//             TEST_FAIL_MESSAGE("DMA transfer timeout");
//             break;
//         }
//         oscl::delay_ms(1);
//     }

//     // 验证发送已完成
//     TEST_ASSERT_TRUE(tx_callback_called);
//     printf("DMA Tx test completed successfully\n");

//     // 清理
//     TestUart::deinit(); 
// }

TEST_CASE(DmaRxModeTest) {
    oscl::delay_ms(3000);
    printf("\nTesting DMA Rx mode...\n");
    // 初始化测试数据
    const uint8_t test_data1[] = "UARTOHDNIPHBIY";
    const uint8_t test_data2[] = "ABCDBNDNIPHBIYUIOGHJKHJK123456";
    const uint8_t test_data3[] = "ABHD";
    const size_t data_len1 = sizeof(test_data1) - 1; // 不包括结尾的\0
    const size_t data_len2 = sizeof(test_data2) - 1; // 不包括结尾的\0
    const size_t data_len3 = sizeof(test_data3) - 1; // 不包括结尾的\0  
    std::span<uint8_t> rx_data1{rx_buffer.data(), data_len1};
    std::span<uint8_t> rx_data2{rx_buffer.data(), data_len2};
    std::span<uint8_t> rx_data3{rx_buffer.data(), data_len3};
    // 打印数据地址
    printf("test_data1: %p\n", test_data1);
    printf("test_data2: %p\n", test_data2);
    printf("test_data3: %p\n", test_data3);
    tx_callback_called = false;
    rx_callback_called = false;
    rx_buffer.fill(0);
    
    // 设置接收完成回调
    TestUart::set_rx_callback(rx_callback);
    
    // ===== 第一次接收测试 =====
    printf("Starting first DMA reception for %d bytes\n", data_len1);
    auto status = TestUart::start_receive(data_len1);
    TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
    
    // 发送第一组测试数据
    TestUart::send(std::span{test_data1, data_len1});
    
    // 等待接收完成
    uint32_t timeout = 1000; // 1秒超时
    uint32_t start = oscl::get_tick_ms();
    while (!rx_callback_called) {
        if (oscl::get_tick_ms() - start > timeout) {
            TEST_FAIL_MESSAGE("First DMA reception timeout");
            break;
        }
        oscl::delay_ms(1);
    }
    
    // 验证第一次接收已完成
    TEST_ASSERT_TRUE(rx_callback_called);
    
    // 读取第一次接收数据
    size_t received1 = TestUart::read(rx_data1);
    printf("rx_data1: %s\n", rx_data1.data());
    TEST_ASSERT_EQUAL(data_len1, received1);
    TEST_ASSERT_EQUAL_MEMORY(test_data1, rx_buffer.data(), data_len1);

    oscl::delay_ms(200);
    
    // ===== 第二次接收测试 =====
    // 重置状态准备第二次接收
    tx_callback_called = false;
    rx_callback_called = false;
    rx_buffer.fill(0);
    
    printf("Starting second DMA reception for %d bytes\n", data_len2);
    
    // 发送第二组测试数据
    TestUart::send(std::span{test_data2, data_len2});

    // 等待第二次接收完成
    start = oscl::get_tick_ms();
    while (!rx_callback_called) {
        if (oscl::get_tick_ms() - start > timeout) {
            TEST_FAIL_MESSAGE("Second DMA reception timeout");
            break;
        }
        oscl::delay_ms(1);
    }
    
    // 验证第二次接收已完成
    TEST_ASSERT_TRUE(rx_callback_called);
    oscl::delay_ms(1000);
    // 读取第二次接收数据

    size_t received2 = TestUart::read(rx_data2);
    printf("rx_data2: %s\n", rx_data2.data());
    TEST_ASSERT_EQUAL(data_len2, received2);
    TEST_ASSERT_EQUAL_MEMORY(test_data2, rx_buffer.data(), data_len2);

    oscl::delay_ms(200);
    
    // ===== 第三次接收测试 =====
    // 重置状态准备第三次接收
    tx_callback_called = false;
    rx_callback_called = false;
    rx_buffer.fill(0);
    
    printf("Starting third DMA reception for %d bytes\n", data_len3);
    // 发送第三组测试数据
    TestUart::send(std::span{test_data3, data_len3});
    
    // 等待第三次接收完成
    start = oscl::get_tick_ms();
    while (!rx_callback_called) {
        if (oscl::get_tick_ms() - start > timeout) {
            TEST_FAIL_MESSAGE("Third DMA reception timeout");
            break;
        }
        oscl::delay_ms(1);
    }
    
    // 验证第三次接收已完成
    TEST_ASSERT_TRUE(rx_callback_called);
    oscl::delay_ms(1000);
    // 读取第三次接收数据
    size_t received3 = TestUart::read(rx_data3);
    printf("rx_data3: %s\n", rx_data3.data());
    TEST_ASSERT_EQUAL(data_len3, received3);
    TEST_ASSERT_EQUAL_MEMORY(test_data3, rx_buffer.data(), data_len3);
    printf("Second DMA reception completed successfully\n");
    printf("DMA Rx ring buffer test passed\n");
    
    // 清理
    TestUart::stop_receive();
}

// TEST_CASE(DmaLoopbackTest) {
//     printf("\nTesting DMA loopback mode...\n");
    
//     if constexpr (test::UartTraits<TestUart>::kTxMode == hcl::UartMode::kDma && 
//                   test::UartTraits<TestUart>::kRxMode == hcl::UartMode::kDma) {
//         // 设置回调
//         TestUart::set_tx_callback(tx_callback);
//         TestUart::set_rx_callback(rx_callback);
        
//         // 初始化测试数据
//         const uint8_t test_data[] = "UART DMA LOOPBACK TEST";
//         const size_t data_len = sizeof(test_data) - 1; // 不包括结尾的\0
//         rx_buffer.fill(0);
//         reset_flags();
        
//         // 启动DMA接收
//         if constexpr (test::UartTraits<TestUart>::kRxTrigger == hcl::UartTrigger::kIdleDetect) {
//             printf("DMA loopback test with idle detect trigger\n");
//             // 空闲检测模式测试
//             auto recv_status = TestUart::start_receive();
//             TEST_ASSERT_EQUAL(hcl::Status::kOk, recv_status);
//         } else {
//             printf("DMA loopback test with normal trigger\n");
//             // 普通触发模式
//             auto recv_status = TestUart::start_receive(data_len);
//             TEST_ASSERT_EQUAL(hcl::Status::kOk, recv_status);
//         }
        
//         // 使用DMA发送数据
//         auto status = TestUart::send(std::span{test_data, data_len});
//         TEST_ASSERT_EQUAL(hcl::Status::kOk, status);
        
//         // 等待发送完成
//         TEST_ASSERT_TRUE(wait_for_condition(tx_callback_called));
        
//         // 等待接收完成
//         TEST_ASSERT_TRUE(wait_for_condition(rx_callback_called));
        
//         // 验证接收数据
//         std::span<uint8_t> rx_data{rx_buffer.data(), data_len};
//         size_t received = TestUart::read(rx_data);
//         TEST_ASSERT_EQUAL(data_len, received);
//         TEST_ASSERT_EQUAL_MEMORY(test_data, rx_buffer.data(), data_len);
        
//         printf("DMA loopback test completed successfully\n");
//     } else {
//         printf("Skipping DMA loopback test - UART not configured for DMA mode\n");
//         TEST_IGNORE_MESSAGE("UART not configured for DMA mode");
//     }
    
//     // 清理
//     TestUart::stop_receive();
// }

TEST_GROUP_RUNNER(TestUart) {
    // RUN_TEST_CASE(TestUart, Loopback);
    // RUN_TEST_CASE(TestUart, EdgeCases);
    // RUN_TEST_CASE(TestUart, BaudRateAccuracy);
    // RUN_TEST_CASE(TestUart, NoiseResistance);
    // RUN_TEST_CASE(TestUart, IdleLineDetection);
    // RUN_TEST_CASE(TestUart, LargeDataTransfer); //收发缓冲区必须大于等于发送数据大小
    // RUN_TEST_CASE(TestUart, ParityModeTest);
    // RUN_TEST_CASE(TestUart, StopBitsTest);
    // RUN_TEST_CASE(TestUart, DmaTxModeTest);
    // RUN_TEST_CASE(TestUart, DmaRxModeTest);
    // RUN_TEST_CASE(TestUart, DmaLoopbackTest);
}


void RunAllTestsForTestUart(void) {
    RUN_TEST_GROUP(TestUart);
}