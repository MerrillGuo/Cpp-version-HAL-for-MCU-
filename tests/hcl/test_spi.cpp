#include "test_spi.h"

#include "board_config.h"
#include "oscl_common.h"
#include "unity_fixture.h"

#include <array>
#include <span>
#include <string_view>

#define LOG_TAG "test_spi"
#define LOG_LEVEL ELOG_LVL_DEBUG
#include "elog.h"

// 使用板载定义的两个SPI设备
using MasterSpi = ImuSpi;     // SPI0 - 阻塞模式
using SlaveSpi = EncoderSpi;  // SPI1 - 中断模式

// 使用板载定义的片选引脚 - 只使用从设备片选引脚
using SlaveCsPin = ImuSpiCsPin;  // 从设备片选引脚

// 片选控制函数 - 修改为主设备控制从设备的片选
void select_slave() {
  SlaveCsPin::clear();  // 主设备拉低从设备的片选（低电平有效）
}

void deselect_slave() {
  SlaveCsPin::set();  // 主设备拉高从设备的片选（高电平无效）
}

// 测试数据
constexpr std::array<uint8_t, 16> kTestPattern = {
    0x55, 0xAA, 0x00, 0xFF, 0x01, 0x02, 0x04, 0x08,
    0x10, 0x20, 0x40, 0x80, 0xF0, 0x0F, 0xC3, 0x3C};

constexpr std::array<uint16_t, 8> kTestWordPattern = {
    0x55AA, 0x00FF, 0x0102, 0x0408, 0x1020, 0x4080, 0xF00F, 0xC33C};

std::array<uint8_t, 64> master_rx_buffer{};
std::array<uint8_t, 64> slave_rx_buffer{};
std::array<uint16_t, 32> master_rx_word_buffer{};
std::array<uint16_t, 32> slave_rx_word_buffer{};

// 添加大数据量测试用的缓冲区
constexpr size_t kLargeBufferSize = 512;  // 减小到512字节以避免超出缓冲区大小

// 执行SPI环回测试
// 修改为字节交替传输模式：从设备发一个字节，主设备发一个字节，然后各自读取
void do_loopback_test(std::span<const uint8_t> tx_data,
                      std::span<uint8_t> master_rx_data,
                      std::span<uint8_t> slave_rx_data) {
  // 选择从设备
  select_slave();
  for (size_t i = 0; i < 10; i++) {
    __NOP();
  }
  __NOP();
  __NOP();
  __NOP();
  // 清空接收缓冲区
  std::fill(master_rx_data.begin(), master_rx_data.end(), 0);
  std::fill(slave_rx_data.begin(), slave_rx_data.end(), 0);

  // 验证数据大小
  size_t transfer_size = std::min(
      tx_data.size(), std::min(master_rx_data.size(), slave_rx_data.size()));
  if (transfer_size == 0) {
    log_e("Invalid transfer size: 0");
    return;
  }
  // 逐字节交替传输
  for (size_t i = 0; i < transfer_size; i++) {
    // 准备单字节传输
    uint8_t master_tx_byte = tx_data[i];
    uint8_t slave_tx_byte =
        tx_data[i];  // 从设备发送相同数据，也可以使用不同数据
    uint8_t master_rx_byte = 0;
    uint8_t slave_rx_byte = 0;

    hcl::Status slave_status = SlaveSpi::transmit(std::span{&slave_tx_byte, 1});
    TEST_ASSERT_EQUAL_MESSAGE(hcl::Status::kOk, slave_status,
                              "Slave transmit failed");

    hcl::Status master_status =
        MasterSpi::transmit(std::span{&master_tx_byte, 1});
    TEST_ASSERT_EQUAL_MESSAGE(hcl::Status::kOk, master_status,
                              "Master transmit failed");

    slave_status = SlaveSpi::receive(std::span{&slave_rx_byte, 1});
    TEST_ASSERT_EQUAL_MESSAGE(hcl::Status::kOk, slave_status,
                              "Slave receive failed");

    master_status = MasterSpi::receive(std::span{&master_rx_byte, 1});
    TEST_ASSERT_EQUAL_MESSAGE(hcl::Status::kOk, master_status,
                              "Master receive failed");

    // 保存接收到的数据
    master_rx_data[i] = master_rx_byte;
    slave_rx_data[i] = slave_rx_byte;
  }

  deselect_slave();

  // 传输完成后验证所有数据
  TEST_ASSERT_EQUAL_MEMORY_MESSAGE(tx_data.data(), master_rx_data.data(), transfer_size, "Master RX mismatch");
  TEST_ASSERT_EQUAL_MEMORY_MESSAGE(tx_data.data(), slave_rx_data.data(), transfer_size, "Slave RX mismatch");

  // 输出测试结果日志
  log_i("Completed byte-by-byte loopback test, %u bytes transferred",
        transfer_size);
}

TEST_GROUP(TestSpi);

TEST_SETUP(TestSpi) {
  master_rx_buffer.fill(0);  // 清空接收缓冲区
  slave_rx_buffer.fill(0);
  master_rx_word_buffer.fill(0);
  slave_rx_word_buffer.fill(0);

  // 初始化片选引脚 - 只初始化从设备的片选引脚
  SlaveCsPin::init(hcl::GpioMode::kOutputPushPull);
  deselect_slave();  // 初始状态为非选中

  // 初始化主SPI (ImuSpi)
  MasterSpi::init()
      .clock_hz(1000000)  // 1MHz
      .mode(hcl::SpiMode::kMode0)
      .bit_order(hcl::SpiBitOrder::kMsbFirst)
      .data_size(hcl::SpiDataSize::kBits8)
      .transfer_type(hcl::SpiTransferType::kFullDuplex)
      .is_master(true);

  // 初始化从SPI (EncoderSpi)
  SlaveSpi::init()
      .clock_hz(1000000)  // 1MHz
      .mode(hcl::SpiMode::kMode0)
      .bit_order(hcl::SpiBitOrder::kMsbFirst)
      .data_size(hcl::SpiDataSize::kBits8)
      .transfer_type(hcl::SpiTransferType::kFullDuplex)
      .is_master(false);
}

TEST_TEAR_DOWN(TestSpi) {
  deselect_slave();
  MasterSpi::deinit();
  SlaveSpi::deinit();
}

// 定义所有测试用例
#define TEST_CASE(name) TEST(TestSpi, name)

TEST_CASE(BasicLoopback) {
  for (size_t i = 0; i < 3; i++) {
    log_i("Testing basic SPI loopback (Master transmit -> Slave receive)");
    do_loopback_test(std::span{kTestPattern},
                     std::span{master_rx_buffer.data(), kTestPattern.size()},
                     std::span{slave_rx_buffer.data(), kTestPattern.size()});
    oscl::delay_ms(10);
  }
}

TEST_CASE(SingleByteTransfer) {
  oscl::delay_ms(10);
  log_i("Testing SPI single byte transfer...");

  // 单字节传输测试
  constexpr std::array<uint8_t, 1> single_byte = {0xA5};
  std::array<uint8_t, 1> master_rx_byte = {0};
  std::array<uint8_t, 1> slave_rx_byte = {0};

  do_loopback_test(std::span{single_byte}, std::span{master_rx_byte},
                   std::span{slave_rx_byte});
}

TEST_GROUP_RUNNER(TestSpi) {
  RUN_TEST_CASE(TestSpi, BasicLoopback);
  RUN_TEST_CASE(TestSpi, SingleByteTransfer);
}

void RunAllTestsForTestSpi(void) {
  RUN_TEST_GROUP(TestSpi);
}