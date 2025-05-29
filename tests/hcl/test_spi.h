#ifndef TESTS_HCL_TEST_SPI_H_
#define TESTS_HCL_TEST_SPI_H_

#include "board_config.h"


#ifdef __cplusplus
extern "C" {
#endif

// 运行SPI测试的入口函数
void RunAllTestsForTestSpi(void);

#ifdef __cplusplus
}
#endif

/*
SPI测试接线说明：

在本测试中，我们使用两个SPI设备进行回环测试：
1. MasterSpi (SPI0) - 配置为主模式，使用阻塞传输模式
2. SlaveSpi (SPI1) - 配置为从模式，使用中断传输模式

硬件连接要求：
- 连接 MasterSpi SCK (PB3)  -> SlaveSpi SCK (PB13)
- 连接 MasterSpi MOSI (PB5) -> SlaveSpi MOSI (PB15)
- 连接 MasterSpi MISO (PB4) -> SlaveSpi MISO (PB14)
- PE8 (EncoderSpiCsPin) 用作从设备的片选信号，由主设备控制

正确的连接是：
- 主设备SPI0: SCK(PB3), MOSI(PB5), MISO(PB4)
- 从设备SPI1: SCK(PB13), MOSI(PB15), MISO(PB14)
- 主设备控制从设备片选: PE8 (EncoderSpiCsPin)
- ImuSpiCsPin (PD4) 不参与测试

注意: 由于两个SPI现在使用不同的物理引脚，确保正确连接对应的引脚进行测试。
*/

#endif  // TESTS_HCL_TEST_SPI_H_ 