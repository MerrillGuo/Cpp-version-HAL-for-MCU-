#include "test_i2c.h"
#include "board_config.h"
#include "oscl_common.h"
#include "unity_fixture.h"
#include <stdio.h>
#include "elog.h"


// 包含 i2c 实现头文件
#include "../bsp/q06/hcl_gd32f3/i2c.h" 

#include <array>
#include <vector>
#include <cstdio>

namespace {

// Test data pattern
constexpr std::array<uint8_t, 8> kWriteTestData = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78};
std::array<uint8_t, kWriteTestData.size()> read_buffer{};

// Flag for interrupt completion signalling
static volatile bool i2c_transfer_complete = false;

} // namespace

TEST_GROUP(TestI2c);

TEST_SETUP(TestI2c) {
    // // Directly use the template instance type
    // using I2cType = hcl::gd32f3::I2c<1>;
    // // Configure and initialize I2C, ENSURE INTERRUPTS ARE ENABLED
    // hcl::Status init_status = I2cType::init(
    //     I2cType::Initializer()
    //         .speed(hcl::I2cSpeed::Standard)
    //         .duty_cycle(hcl::I2cDutyCycle::Duty2)
    //         .addr_mode(hcl::I2cAddressingMode::k7bit)
    //         .dma_mode(hcl::I2cDmaMode::kDisable)
    //         .event_int_mode(hcl::I2cEventInterruptMode::kDisable) // <<< DISABLE EVENT INTERRUPT
    //         .error_int_mode(hcl::I2cErrorInterruptMode::kDisable) // <<< DISABLE ERROR INTERRUPT
    // );

    // TEST_ASSERT_EQUAL(hcl::Status::kOk, init_status);
    // oscl::delay_ms(5);
    // I2cType::clearError();
    // read_buffer.fill(0);
    i2c_transfer_complete = false; // Reset flag before each test
}

TEST_TEAR_DOWN(TestI2c) {
    // Directly use the template instance type
    using I2cType = hcl::gd32f3::I2c<1>;
    I2cType::deinit();
    oscl::delay_ms(5); 
}

// // Test Case 1: Write Operation
// TEST(TestI2c, Write) {
//     using I2cType = hcl::gd32f3::I2c<1>;
//     printf("Testing I2C Write to Addr 0x%X, Reg 0x%X...\n", kTestSlaveAddr, kTestRegAddr);
//     hcl::I2cStatus write_status = I2cType::write_reg(kTestSlaveAddr, kTestRegAddr, kWriteTestData.data(), kWriteTestData.size());
    
//     TEST_ASSERT_EQUAL(hcl::I2cStatus::Ok, write_status);
//     printf("Write test completed (Status OK)\n");
// }

// // Test Case 2: Read Operation (after Write)
// TEST(TestI2c, Read) {
//     using I2cType = hcl::gd32f3::I2c<1>;
//     printf("Testing I2C Read from Addr 0x%X, Reg 0x%X...\n", kTestSlaveAddr, kTestRegAddr);

//     // 1. Write the data first
//     hcl::I2cStatus write_status = I2cType::write_reg(kTestSlaveAddr, kTestRegAddr, kWriteTestData.data(), kWriteTestData.size());
//     TEST_ASSERT_EQUAL(hcl::I2cStatus::Ok, write_status);
//     oscl::delay_ms(2); 

//     // 2. Read the data back
//     hcl::I2cStatus read_status = I2cType::read_reg(kTestSlaveAddr, kTestRegAddr, read_buffer.data(), read_buffer.size());
//     TEST_ASSERT_EQUAL(hcl::I2cStatus::Ok, read_status);

//     // 3. Verify the read data matches the written data
//     TEST_ASSERT_EQUAL_MEMORY(kWriteTestData.data(), read_buffer.data(), kWriteTestData.size());
//     printf("Read test completed (Data verified)\n");
// }

// Test Case 3: Read 16-bit Register
TEST(TestI2c, Read16BitRegister) {
    // Directly use the template instance type
    using I2cType = hcl::gd32f3::I2c<1>;
    // Configure and initialize I2C, ENSURE INTERRUPTS ARE ENABLED
    hcl::Status init_status = I2cType::init(
        I2cType::Initializer()
            .speed(hcl::I2cSpeed::Standard)
            .duty_cycle(hcl::I2cDutyCycle::Duty2)
            .addr_mode(hcl::I2cAddressingMode::k7bit)
            .dma_mode(hcl::I2cDmaMode::kDisable)
            .event_int_mode(hcl::I2cEventInterruptMode::kDisable) // <<< DISABLE EVENT INTERRUPT
            .error_int_mode(hcl::I2cErrorInterruptMode::kDisable) // <<< DISABLE ERROR INTERRUPT
    );

    TEST_ASSERT_EQUAL(hcl::Status::kOk, init_status);
    oscl::delay_ms(5);
    I2cType::clearError();
    read_buffer.fill(0);

    constexpr uint16_t slave_addr = 0x16;
    constexpr uint16_t reg_addr = 0x19;
    std::array<uint8_t, 2> read_buf_16bit{}; 

    log_i("Testing I2C Read 16-bit from Addr 0x%X, Reg 0x%X...\n", slave_addr, reg_addr);
    while(1) {
        I2cType::read_reg(slave_addr, reg_addr, read_buf_16bit.data(), read_buf_16bit.size());
        oscl::delay_ms(1000);
        // Combine bytes (Assuming MSB first)
        uint16_t value_16bit = (static_cast<uint16_t>(read_buf_16bit[1]) << 8) |
                            static_cast<uint16_t>(read_buf_16bit[0]);

        log_i("Read 16-bit value: 0x%04X (%u)\n", value_16bit, value_16bit);
    }
    // Read 2 bytes
    hcl::I2cStatus read_status = I2cType::read_reg(slave_addr, reg_addr, read_buf_16bit.data(), read_buf_16bit.size());
    TEST_ASSERT_EQUAL(hcl::I2cStatus::Ok, read_status);

    // Combine bytes (Assuming MSB first)
    uint16_t value_16bit = (static_cast<uint16_t>(read_buf_16bit[1]) << 8) |
                           static_cast<uint16_t>(read_buf_16bit[0]);

    log_i("Read 16-bit value: 0x%04X (%u)\n", value_16bit, value_16bit);
    oscl::delay_ms(1000);
}

// Test Case 4: Read Operation using Interrupts
TEST(TestI2c, ReadInterrupt) {

    // Directly use the template instance type
    using I2cType = hcl::gd32f3::I2c<1>;
    // Configure and initialize I2C, ENSURE INTERRUPTS ARE ENABLED
    hcl::Status init_status = I2cType::init(
        I2cType::Initializer()
            .speed(hcl::I2cSpeed::Standard)
            .duty_cycle(hcl::I2cDutyCycle::Duty2)
            .addr_mode(hcl::I2cAddressingMode::k7bit)
            .dma_mode(hcl::I2cDmaMode::kDisable)
            .event_int_mode(hcl::I2cEventInterruptMode::kEnable) // <<< DISABLE EVENT INTERRUPT
            .error_int_mode(hcl::I2cErrorInterruptMode::kDisable) // <<< DISABLE ERROR INTERRUPT
    );

    TEST_ASSERT_EQUAL(hcl::Status::kOk, init_status);
    oscl::delay_ms(5);
    I2cType::clearError();
    read_buffer.fill(0);

    constexpr uint16_t slave_addr = 0x16; // Use the same address as other tests
    constexpr uint16_t reg_addr = 0x19;   // Use the same register as other tests
    constexpr size_t read_length = 2;             // Let's read 2 bytes for testing
    std::array<uint8_t, read_length> read_buf_interrupt{};
    read_buf_interrupt.fill(0xFF); // Pre-fill to ensure it gets overwritten

    // --- Step 2: Set up callback --- 
    i2c_transfer_complete = false; // Ensure flag is reset
    I2cType::set_event_callback([]() {
        i2c_transfer_complete = true;
    });

    // --- Step 3: Initiate non-blocking read --- 
    hcl::I2cStatus read_start_status = I2cType::read_reg_interrupt(
        slave_addr, 
        static_cast<uint8_t>(reg_addr), // Cast reg_addr if needed by function signature 
        read_buf_interrupt.data(), 
        read_buf_interrupt.size()
    );
    TEST_ASSERT_EQUAL(hcl::I2cStatus::Ok, read_start_status); // Check if the read *started* ok
    log_i("  Interrupt read initiated. Waiting for completion...\n");

    // --- Step 4: Wait for completion via callback flag (with timeout) --- 
    uint32_t timeout_ms = 1000; // 1000ms timeout for interrupt transfer
    uint32_t start_tick = oscl::get_tick_ms();
    while (!i2c_transfer_complete) {
        if (oscl::get_tick_ms() - start_tick > timeout_ms) {
            I2cType::set_event_callback(nullptr); // Clear callback on timeout
            TEST_FAIL_MESSAGE("I2C Interrupt Read Timeout");
            return; // Exit test on failure
        }
    }

   // Combine bytes (Assuming MSB first)
    uint16_t value_16bit = (static_cast<uint16_t>(read_buf_interrupt[1]) << 8) |
                           static_cast<uint16_t>(read_buf_interrupt[0]);

    log_i("Read 16-bit value: 0x%04X (%u)\n", value_16bit, value_16bit);
}

TEST_GROUP_RUNNER(TestI2c) {
    // RUN_TEST_CASE(TestI2c, Write);
    // RUN_TEST_CASE(TestI2c, Read);
    // RUN_TEST_CASE(TestI2c, Read16BitRegister);
    RUN_TEST_CASE(TestI2c, ReadInterrupt); // Add the new test case
}

extern "C" void RunAllTestsForTestI2c(void) {
    RUN_TEST_GROUP(TestI2c);
}
