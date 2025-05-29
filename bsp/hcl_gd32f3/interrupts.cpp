#include "board_config.h"

using namespace hcl::gd32f3;

#ifdef __cplusplus
extern "C" {
#endif

void USART0_IRQHandler(void) {
    Uart<0>::irq_handler();
}

void USART1_IRQHandler(void) {
    Uart<1>::irq_handler();
}

void USART2_IRQHandler(void) {
    Uart<2>::irq_handler();
}

void UART3_IRQHandler(void) {
    Uart<3>::irq_handler();
}

void UART4_IRQHandler(void) {
    Uart<4>::irq_handler();
}

// Timer0 相关中断
void TIMER0_BRK_TIMER8_IRQHandler(void) {
    Timer<8>::irq_handler();  // Timer8 中断
}

void TIMER0_UP_TIMER9_IRQHandler(void) {
    Timer<0>::irq_handler();  // Timer0 更新中断
    Timer<9>::irq_handler();  // Timer9 中断
}

void TIMER0_TRG_CMT_TIMER10_IRQHandler(void) {
    Timer<10>::irq_handler();  // Timer10 中断
}

// ADC 相关中断
void ADC0_1_IRQHandler(void) {
    Adc<0>::irq_handler();
    Adc<1>::irq_handler();
}

void ADC2_IRQHandler(void) {
    Adc<2>::irq_handler();
}

// 独立定时器中断
void TIMER1_IRQHandler(void) {
    Timer<1>::irq_handler();
}

void TIMER2_IRQHandler(void) {
    Timer<2>::irq_handler();
}

void TIMER3_IRQHandler(void) {
    Timer<3>::irq_handler();
}

void TIMER4_IRQHandler(void) {
    Timer<4>::irq_handler();
}

void TIMER5_IRQHandler(void) {
    Timer<5>::irq_handler();
}

void TIMER6_IRQHandler(void) {
    Timer<6>::irq_handler();
}

// Timer7 相关中断
void TIMER7_BRK_TIMER11_IRQHandler(void) {
    Timer<11>::irq_handler();  // Timer11 中断
}

void TIMER7_UP_TIMER12_IRQHandler(void) {
    Timer<7>::irq_handler();   // Timer7 更新中断
    Timer<12>::irq_handler();  // Timer12 中断
}

void TIMER7_TRG_CMT_TIMER13_IRQHandler(void) {
    Timer<13>::irq_handler();  // Timer13 中断
}

void SPI0_IRQHandler(void) {
    Spi<0>::irq_handler();
}

void SPI1_IRQHandler(void) {
    Spi<1>::irq_handler();
}

// DMA 相关中断
void DMA0_Channel0_IRQHandler(void) {
    Dma<0, 0>::irq_handler();
}

void DMA0_Channel1_IRQHandler(void) {
    Dma<0, 1>::irq_handler();
}

void DMA0_Channel2_IRQHandler(void) {
    Dma<0, 2>::irq_handler();
}

void DMA0_Channel3_IRQHandler(void) {
    Dma<0, 3>::irq_handler();
}

void DMA0_Channel4_IRQHandler(void) {
    Dma<0, 4>::irq_handler();
}

void DMA0_Channel5_IRQHandler(void) {
    Dma<0, 5>::irq_handler();
}

void DMA0_Channel6_IRQHandler(void) {
    Dma<0, 6>::irq_handler();
}

void DMA1_Channel0_IRQHandler(void) {
    Dma<1, 0>::irq_handler();
}

void DMA1_Channel1_IRQHandler(void) {
    Dma<1, 1>::irq_handler();
}

void DMA1_Channel2_IRQHandler(void) {
    Dma<1, 2>::irq_handler();
}   

void DMA1_Channel3_4_IRQHandler(void) {
    Dma<1, 3>::irq_handler();
    Dma<1, 4>::irq_handler();
}   

// EXTI 中断处理函数
void EXTI0_IRQHandler(void) {
    handle_exti(0);
}

void EXTI1_IRQHandler(void) {
    handle_exti(1);
}

void EXTI2_IRQHandler(void) {
    handle_exti(2);
}

void EXTI3_IRQHandler(void) {
    handle_exti(3);
}

void EXTI4_IRQHandler(void) {
    handle_exti(4);
}

void EXTI5_9_IRQHandler(void) {
    for (uint8_t i = 5; i <= 9; i++) {
        handle_exti(i);
    }
}

void EXTI10_15_IRQHandler(void) {
    for (uint8_t i = 10; i <= 15; i++) {
        handle_exti(i);
    }
}

void I2C0_EV_IRQHandler(void) {
    I2c<0>::irq_handler();
}

void I2C0_ER_IRQHandler(void) {
    I2c<0>::irq_handler();
}

void I2C1_EV_IRQHandler(void) {
    I2c<1>::irq_handler();
}

void I2C1_ER_IRQHandler(void) {
    I2c<1>::irq_handler();
}


#ifdef __cplusplus
}
#endif