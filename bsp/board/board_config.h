#ifndef BSP_Q06_BOARD_BOARD_CONFIG_H_
#define BSP_Q06_BOARD_BOARD_CONFIG_H_

#include "gpio.h"
#include "uart.h"
#include "timer.h"
#include "spi.h"
#include "adc.h"
#include "dma.h"
#include "micros.h"
#include "exti.h"
#include "i2c.h"

namespace hcl {
namespace gd32f3 {

using GlobalMicrosTimer = MicrosTimer<Timer<5>>;

// GPIO definitions
// using Led0 = PA<1>;
// using Led1 = PA<2>;

using Power5VEn = PE<0>;
using Power3V3En = PE<1>;
using Power16VEn = PE<12>;
using PowerHostEn = PG<0>;

using FrontLeftBumperPin = PC<13>;
using FrontRightBumperPin = PE<4>;
using RearLeftBumperPin = PA<12>;
using RearRightBumperPin = PA<11>;

using LidarLiftMotorForwardPin = PD<12>;
using LidarLiftMotorReversePin = PD<13>;
using LidarLiftTopPin = PF<1>;
using LidarLiftBottomPin = PF<2>;

using ImuSpiCsPin = PD<4>;

// 传感器 IO
using SewageTankFullDetectPin = PA<3>;
using CleanTankFullDetectPin = PA<8>;
using CoverInplacePin = PD<11>;
using FrontBrushInplacePin = PE<2>;
using BackBrushInplacePin = PE<3>;
using SewageTankInplacePin = PE<10>;
using FrontWaterPipeEmptyPin = PF<11>;
using BackWaterPipeEmptyPin = PF<12>;
using StickInplacePin = PF<13>;
using LeftPyroelectricPin = PG<2>;
using RightPyroelectricPin = PG<3>;

using LeftDriveOffGroundPin = PD<7>;
using RightDriveOffGroundPin = PD<10>;

using SewageTankFullDrive1Pin = PG<4>;
using SewageTankFullDrive2Pin = PG<5>;

using HomeButtonPin = PF<14>;
using StartButtonPin = PF<15>;

using ButtonLedStripTimer = Timer<4>;
static constexpr uint8_t kButtonLedStripTimerChannel = 1;
static constexpr uint8_t kButtonLedCount = 3;

using ExteriorLedStripTimer = Timer<4>;
static constexpr uint8_t kExteriorLedStripTimerChannel = 2;
static constexpr uint8_t kExteriorLedCount = 16;


// 电机GPIO定义
using FanPowerEn = PE<7>;
using FanTimerType = Timer<2>;
static constexpr uint8_t kFanForwardChannel = 0;

using FrontBrushTimerType = Timer<0>;
static constexpr uint8_t kFrontBrushForwardChannel = 1;
static constexpr uint8_t kFrontBrushReverseChannel = 0;

using BackBrushTimerType = Timer<0>;
static constexpr uint8_t kBackBrushForwardChannel = 2;
static constexpr uint8_t kBackBrushReverseChannel = 3;

using FrontPumpGpioPin = PD<14>;
using BackPumpGpioPin = PD<15>;

using CameraLightEnPin = PA<0>;

using LeftIrExti = ExtiPB<8>;
using RightIrExti = ExtiPB<9>;

using IrTimer = GlobalMicrosTimer;

// 悬崖检测对管和脏污检测对管
using IrPairTimer = Timer<2>;
static constexpr uint8_t kCliffIrPairChannel = 1;
static constexpr uint8_t kSewageIrPairChannel = 3;

using CliffIrPairAdc = Adc<0>;
static constexpr uint8_t kCliffFrontRightAdcChannel = 6;
static constexpr uint8_t kCliffFrontLeftAdcChannel = 7;
static constexpr uint8_t kCliffBackLeftAdcChannel = 8;
static constexpr uint8_t kCliffBackRightAdcChannel = 9;

using SewageIrPairAdc = Adc<2>;
static constexpr uint8_t kSewageAdcChannel = 4;

// SPI 配置特化
template<>
struct SpiConfig<0> {  // SPI0 配置
    using SckPin = PB<3>;
    using MosiPin = PB<5>;
    using MisoPin = PB<4>;
    using SsPin = void;
    
    static constexpr SpiTransferMode kTransferMode = SpiTransferMode::kBlocking;
    static constexpr SpiTransferType kTransferType = SpiTransferType::kFullDuplex;
    static constexpr size_t kRxBufSize = 128;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
    static constexpr bool hardware_ss = false;  // 使用软件控制片选
};

template<>
struct SpiConfig<1> {  // SPI1 配置
    using SckPin = PB<13>;
    using MosiPin = PB<15>;
    using MisoPin = PB<14>;
    // using SsPin = PB<12>;
    using SsPin = void;
    
    static constexpr SpiTransferMode kTransferMode = SpiTransferMode::kBlocking;
    static constexpr SpiTransferType kTransferType = SpiTransferType::kFullDuplex;
    static constexpr size_t kRxBufSize = 64;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
    static constexpr bool hardware_ss = false; 
};

using EncoderSpiCsPin = PE<8>;

// 定义SPI实例
using ImuSpi = Spi<0>;
using EncoderSpi = Spi<1>;

template<size_t Instance>
struct TimerPwmPinMap;

template<size_t Instance>
struct AdcPinMap;

template<>
struct TimerPwmPinMap<0> {
    template<uint8_t Channel>
    struct Pin {
        using Type = typename std::conditional<Channel==0, PE<9>,
                    typename std::conditional<Channel==1, PE<11>, 
                    typename std::conditional<Channel==2, PE<13>,
                    typename std::conditional<Channel==3, PE<14>, void>::type>::type>::type>::type;
        
        static constexpr GpioMode kMode = GpioMode::kAltPushPull;
        static constexpr GpioSpeed kSpeed = GpioSpeed::kHigh;
        static constexpr uint32_t kPinRemap = GPIO_TIMER0_FULL_REMAP;
    };
};

template<>
struct TimerPwmPinMap<2> {
    template<uint8_t Channel>
    struct Pin {
        using Type = typename std::conditional<Channel==0, PC<6>,
                    typename std::conditional<Channel==1, PC<7>, 
                    typename std::conditional<Channel==2, PC<8>,
                    typename std::conditional<Channel==3, PC<9>, void>::type>::type>::type>::type;
        
        static constexpr GpioMode kMode = GpioMode::kAltPushPull;
        static constexpr GpioSpeed kSpeed = GpioSpeed::kHigh;
        static constexpr uint32_t kPinRemap = GPIO_TIMER2_FULL_REMAP;
    };
};

template<>
struct TimerPwmPinMap<4> {
    template<uint8_t Channel>
    struct Pin {
        using Type =typename std::conditional<Channel==1, PA<1>,
                    typename std::conditional<Channel==2, PA<2>,
                     void>::type>::type;
        
        static constexpr GpioMode kMode = GpioMode::kAltPushPull;
        static constexpr GpioSpeed kSpeed = GpioSpeed::kHigh;
        static constexpr uint32_t kPinRemap = 0;    // 没有重映射
    };
};


template<>
struct I2cConfig<0> {
    using SclPin = PB<6>;
    using SdaPin = PB<7>;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
};

template<>
struct I2cConfig<1> {
    using SclPin = PB<10>;
    using SdaPin = PB<11>;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
};

// ADC引脚映射特化
template<size_t Instance>
struct AdcPinMap;

// ADC0和ADC1的引脚映射
template<>
struct AdcPinMap<0> {
    template<uint8_t Channel>
    struct Pin {
        using Type = typename std::conditional<Channel==0,  PA<0>,  // ADC012_IN0
                    typename std::conditional<Channel==1,  PA<1>,   // ADC012_IN1
                    typename std::conditional<Channel==2,  PA<2>,   // ADC012_IN2
                    typename std::conditional<Channel==3,  PA<3>,   // ADC012_IN3
                    typename std::conditional<Channel==4,  PA<4>,   // ADC01_IN4
                    typename std::conditional<Channel==5,  PA<5>,   // ADC01_IN5
                    typename std::conditional<Channel==6,  PA<6>,   // ADC01_IN6
                    typename std::conditional<Channel==7,  PA<7>,   // ADC01_IN7
                    typename std::conditional<Channel==8,  PB<0>,   // ADC01_IN8
                    typename std::conditional<Channel==9,  PB<1>,   // ADC01_IN9
                    typename std::conditional<Channel==10, PC<0>,   // ADC012_IN10
                    typename std::conditional<Channel==11, PC<1>,   // ADC012_IN11
                    typename std::conditional<Channel==12, PC<2>,   // ADC012_IN12
                    typename std::conditional<Channel==13, PC<3>,   // ADC012_IN13
                    typename std::conditional<Channel==14, PC<4>,   // ADC01_IN14
                    typename std::conditional<Channel==15, PC<5>,   // ADC01_IN15
                    void>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type;
        
        static constexpr GpioMode kMode = GpioMode::kAnalog;
        static constexpr GpioSpeed kSpeed = GpioSpeed::kLow;
    };
};

// ADC1与ADC0使用相同的引脚映射
template<>
struct AdcPinMap<1> : public AdcPinMap<0> {};

// ADC2的引脚映射，仅包含ADC2支持的通道
template<>
struct AdcPinMap<2> {
    template<uint8_t Channel>
    struct Pin {
        // 基于通道选择对应的GPIO引脚
        using Type = 
            // 基本通道 0-3
            typename std::conditional<Channel == 0, PA<0>,  // ADC012_IN0
            typename std::conditional<Channel == 1, PA<1>,  // ADC012_IN1
            typename std::conditional<Channel == 2, PA<2>,  // ADC012_IN2
            typename std::conditional<Channel == 3, PA<3>,  // ADC012_IN3
            
            // ADC2特有通道 4-8
            typename std::conditional<Channel == 4, PF<6>,  // ADC2_IN4
            typename std::conditional<Channel == 5, PF<7>,  // ADC2_IN5
            typename std::conditional<Channel == 6, PF<8>,  // ADC2_IN6
            typename std::conditional<Channel == 7, PF<9>,  // ADC2_IN7
            typename std::conditional<Channel == 8, PF<10>, // ADC2_IN8
            typename std::conditional<Channel == 9, PG<9>, // 无用引脚，拿来垫着
            
            // 基本通道 10-13
            typename std::conditional<Channel == 10, PC<0>, // ADC012_IN10
            typename std::conditional<Channel == 11, PC<1>, // ADC012_IN11
            typename std::conditional<Channel == 12, PC<2>, // ADC012_IN12
            typename std::conditional<Channel == 13, PC<3>, // ADC012_IN13
            typename std::conditional<Channel == 14, PG<14>, // 无用引脚，拿来垫着
            typename std::conditional<Channel == 15, PG<15>, // 无用引脚，拿来垫着
            void>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type;
        
        static constexpr GpioMode kMode = GpioMode::kAnalog;
        static constexpr GpioSpeed kSpeed = GpioSpeed::kLow;
    };
};

// UART配置特化
template<>
struct UartConfig<0> { 
    using TxPin = PA<9>;
    using RxPin = PA<10>;
    static constexpr UartMode kRxMode = UartMode::kInterrupt;
    static constexpr UartMode kTxMode = UartMode::kInterrupt;
    static constexpr UartTrigger kRxTrigger = UartTrigger::kNormal;
    static constexpr size_t kRxBufSize = 32;
    static constexpr size_t kTxBufSize = 32;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
};

template<>
struct UartConfig<1> {
    using TxPin = PD<5>;
    using RxPin = PD<6>;
    static constexpr UartMode kRxMode = UartMode::kInterrupt;
    static constexpr UartMode kTxMode = UartMode::kInterrupt;
    static constexpr UartTrigger kRxTrigger = UartTrigger::kNormal;
    static constexpr size_t kRxBufSize = 32;
    static constexpr size_t kTxBufSize = 32;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
};

template<>
struct UartConfig<2> {
    using TxPin = PD<8>;
    using RxPin = PD<9>;
    static constexpr UartMode kRxMode = UartMode::kInterrupt;
    static constexpr UartMode kTxMode = UartMode::kInterrupt;
    static constexpr UartTrigger kRxTrigger = UartTrigger::kNormal;
    static constexpr size_t kRxBufSize = 30;
    static constexpr size_t kTxBufSize = 30;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
};

template<>
struct UartConfig<3> {
    using TxPin = PC<10>;
    using RxPin = PC<11>;
    static constexpr UartMode kRxMode = UartMode::kInterrupt;
    static constexpr UartMode kTxMode = UartMode::kInterrupt;
    static constexpr UartTrigger kRxTrigger = UartTrigger::kIdleDetect;
    static constexpr size_t kRxBufSize = 256;
    static constexpr size_t kTxBufSize = 256;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
};

template<>
struct UartConfig<4> {
    using TxPin = PC<12>;
    using RxPin = PD<2>;
    static constexpr UartMode kRxMode = UartMode::kInterrupt;
    static constexpr UartMode kTxMode = UartMode::kInterrupt;
    static constexpr UartTrigger kRxTrigger = UartTrigger::kNormal;
    static constexpr size_t kRxBufSize = 64;
    static constexpr size_t kTxBufSize = 64;
    static constexpr uint8_t kPreemptPriority = 6;
    static constexpr uint8_t kSubPriority = 0;
};


// 定义UART实例
using CommUart = Uart<0>;
using LeftMotorUart = Uart<1>;
using RightMotorUart = Uart<2>;
using DebugUart = Uart<3>;
using RcUart = Uart<3>;
using CarpetDetectorUart = Uart<4>;

} // namespace gd32f3
} // namespace hcl

using namespace hcl::gd32f3;

#endif  // BSP_Q06_BOARD_BOARD_CONFIG_H_ 