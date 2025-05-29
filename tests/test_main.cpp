#include "hcl/test_ring_buffer.h"
#include "hcl/test_uart.h"
#include "hcl/test_timer.h"
#include "hcl/test_spi.h"
#include "hcl/test_adc.h"
#include "hcl/test_dma.h"
#include "hcl/test_i2c.h"
#include "modules/test_mcn.h"
#include "modules/test_crsf.h"
#include "modules/test_crsf_mock.h"
#include "modules/test_tiny_link.h"
#include "modules/test_odometry.h"
#include "modules/test_mpu6xxx.h"
#include "modules/test_ws2811.h"    
#include "modules/test_cliff.h"
#include "board_config.h"
#include "oscl_common.h"

#include "unity_fixture.h"

#define LOG_TAG "test_main"
#define LOG_LEVEL ELOG_LVL_DEBUG
#include "elog.h"

void PowerEnable(void) {
    hcl::gpio_init_push_pull_out<Power5VEn, Power3V3En, Power16VEn, PowerHostEn>();
    Power5VEn::set();
    Power3V3En::set();
    Power16VEn::set();
    PowerHostEn::set();
}


void DebugUartErrorCallback(hcl::UartError error) {
    // log_e("Debug UART Error: %d\n", error);
}

void DebugUartInit(void) {
    // 初始化调试串口
    hcl::gd32f3::DebugUart::init().baudrate(115200);
    hcl::gd32f3::DebugUart::set_error_callback(DebugUartErrorCallback);
}

extern "C" __attribute__((used)) void MainTask(void *argument)
{
    (void)argument;

    PowerEnable();

    DebugUartInit();

    UNITY_BEGIN();
    
    // // RingBuffer测试组
    // log_i("Running RingBuffer Tests.\n");
    // RunAllTestsForTestRingBuffer();

    // // // MCN测试组
    // log_i("Running MCN Tests.\n");
    // RunAllTestsForTestMcn();

    // // CRSF mock测试组
    // log_i("Running CRSF Mock Tests.\n");
    // RunAllTestsForTestCrsfMock();

    // TinyLink测试组
    // log_i("Running TinyLink Tests.\n");
    // RunAllTestsForTinyLink();
    
    // // Odometry测试组
    // log_i("Running Odometry Tests.\n");
    // RunAllTestsForTestOdometry();
    
    // WS2811测试组
    // log_i("Running WS2811 Tests.\n");
    // RunAllTestsForTestWS2811();
    // log_i("Running Odometry Tests.\n");
    // RunAllTestsForTestOdometry();


    // ------------------------------
    // 以下测试需要连接真实硬件
    // ------------------------------

    // // UART测试组
    // log_i("Running UART Tests.\n");
    // RunAllTestsForTestUart();

    // Timer测试组
    // log_i("Running Timer Tests.\n");
    // RunAllTestsForTestTimer();

    // MCN测试组
    // log_i("Running MCN Tests.\n");
    // RunAllTestsForTestMcn();
    
    // CRSF hardware测试组
    // log_i("Running CRSF Hardware Tests.\n");
    // RunAllTestsForTestCrsf();
    
    // SPI测试组
    // log_i("Running SPI Tests.\n");
    // RunAllTestsForTestSpi();

    // MPU6xxx测试组
    // log_i("Running MPU6xxx Tests.\n");
    // RunAllTestsForTestMpu6xxx();

    // TinyLink测试组
    // log_i("Running TinyLink Tests.\n");
    // RunAllTestsForTestTinyLink();

    // ADC测试组
    // log_i("Running ADC Tests.\n");
    // RunAllTestsForTestAdc();

    // DMA测试组
    // log_i("Running DMA Tests.\n");
    // RunAllTestsForTestDma();

    // IR Pair测试组
    // log_i("Running IR Pair Tests.\n");
    // RunAllTestsForTestIrPair();
    
    // 悬崖检测测试组
    // log_i("Running Cliff Detection Tests.\n");
    // RunAllTestsForTestCliff();

    // I2C测试组
    // log_i("Running I2C Tests.\n");
    RunAllTestsForTestI2c();

    UNITY_END();
    // 初始化led
    // hcl::gpio_init_push_pull_out<Led1>();

    while (true) {
        // Led1::toggle();
        oscl::delay_ms(1000);
    }
}

