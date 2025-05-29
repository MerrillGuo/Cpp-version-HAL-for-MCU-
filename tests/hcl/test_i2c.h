#ifndef TEST_I2C_H
#define TEST_I2C_H

#include "board_config.h"

// Define a placeholder slave address for testing
// Replace with the actual 7-bit address of your test device
constexpr uint16_t kTestSlaveAddr = 0x16; 
// Define a placeholder register address for read/write tests
constexpr uint16_t kTestRegAddr = 0x18;

#ifdef __cplusplus
extern "C" {
#endif

// Unity Fixture Test Group Runner
void RunAllTestsForTestI2c(void);

// Test setup and teardown functions
void i2c_test_set_up(void);
void i2c_test_tear_down(void);

// Test cases
void test_i2c_write(void);  // Tests basic write or write_reg
void test_i2c_read(void);   // Tests basic read or read_reg after writing
void test_i2c_read_16bit_register(void); // Test reading a 16-bit register

#ifdef __cplusplus
}
#endif

#endif // TEST_I2C_H
