#ifndef HCL_I2C_IMPL_H
#define HCL_I2C_IMPL_H

#include "hcl_i2c.h"
#include "traits/i2c_traits.h"
#include "../board/board_config.h"
#include "gd32f30x.h"
#include "gd32f30x_i2c.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_rcu.h"

#include "gpio.h"
#include "bitband.h"
#include "hcl_common.h"
#include "oscl_common.h"
#include <functional>
#include <cstring>
#include <type_traits>

namespace hcl {
namespace gd32f3 {

template<size_t Instance>
struct I2cConfig;

// Concrete I2C implementation for GD32F30x
template<size_t Instance>
class I2cImpl {
    using Config = I2cConfig<Instance>;
    using Traits = I2cTraits<Instance>;

    static inline I2cEventCallback i2c_event_callback_;

    static_assert(Instance < 2, "Invalid I2C instance number");
    static_assert(Config::kPreemptPriority < 16, "Invalid preempt priority");
    static_assert(Config::kSubPriority < 16, "Invalid sub-priority");

    static void enable() { Traits::i2cen.set(); }
    static void disable() { Traits::i2cen.clear(); }
    static void start_on_bus() { Traits::start.set(); }
    static void stop_on_bus() { Traits::stop.set(); }
    static void ack_enable() { Traits::acken.set(); }
    static void ack_disable() { Traits::acken.clear(); }
    static void send_byte(uint8_t data) { I2C_DATA(Traits::kBase) = data; }
    static uint8_t receive_byte() { return static_cast<uint8_t>(I2C_DATA(Traits::kBase)); }

    static bool _wait_for_flag(i2c_flag_enum flag, FlagStatus state, uint32_t timeout_ms) {
        uint32_t start_tick = oscl::get_tick_ms();
        while (i2c_flag_get(Traits::kBase, flag) != state) {
            if (oscl::get_tick_ms() - start_tick > timeout_ms) {
                return false; 
            }
        }
        return true;
    }

    // --- Specific Wait Functions using Helper ---
    // Note: The timeout parameter passed to these functions now represents milliseconds.
    static I2cStatus wait_bus_idle(uint32_t timeout_ms) {
        return _wait_for_flag(I2C_FLAG_I2CBSY, RESET, timeout_ms) ? I2cStatus::Ok : I2cStatus::Busy; // Timeout on busy is Busy status
    }
    
    static I2cStatus wait_start_sent(uint32_t timeout_ms) {
        return _wait_for_flag(I2C_FLAG_SBSEND, SET, timeout_ms) ? I2cStatus::Ok : I2cStatus::Timeout;
    }
    
    static I2cStatus wait_address_sent(uint32_t timeout_ms) {
        // ADDSEND flag needs STAT0 and STAT1 read to be cleared
        if (!_wait_for_flag(I2C_FLAG_ADDSEND, SET, timeout_ms)) {
            return I2cStatus::Timeout;
        }
        // Clear ADDSEND by reading STAT0 and STAT1 (as per GD32 HAL)
        i2c_flag_clear(Traits::kBase, I2C_FLAG_ADDSEND);
        return I2cStatus::Ok;
    }

    static I2cStatus wait_tbe(uint32_t timeout_ms) {
        return _wait_for_flag(I2C_FLAG_TBE, SET, timeout_ms) ? I2cStatus::Ok : I2cStatus::Timeout;
    }
    
    static I2cStatus wait_rbne(uint32_t timeout_ms) {
        return _wait_for_flag(I2C_FLAG_RBNE, SET, timeout_ms) ? I2cStatus::Ok : I2cStatus::Timeout;
    }
    
    static I2cStatus wait_btc(uint32_t timeout_ms) {
        return _wait_for_flag(I2C_FLAG_BTC, SET, timeout_ms) ? I2cStatus::Ok : I2cStatus::Timeout;
    }
    
    static I2cStatus wait_stop_cleared(uint32_t timeout_ms) {
        return wait_bus_idle(timeout_ms); 
    }

public:
    // Variables for interrupt-driven transfers
    static inline volatile uint8_t* i2c_tx_buffer = nullptr; // Pointer to transmit buffer
    static inline volatile uint8_t* i2c_rx_buffer = nullptr; // Pointer to receive buffer
    static inline volatile uint16_t i2c_tx_nBytes = 0;        // Number of bytes left to transfer
    static inline volatile uint16_t i2c_rx_nBytes = 0;        // Number of bytes left to transfer
    static inline volatile uint16_t slave_device_addr = 0;        // Device address
    static inline volatile uint8_t i2c_tx_reg_addr = 0;        // Register address for write operations
    static inline volatile uint8_t i2c_rx_reg_addr = 0;        // Register address for read operations
    // Potentially add state variables for interrupt FSM (e.g., direction, current step)

    class Initializer : public ::hcl::I2cInitializerBase<Initializer> {
    public:
        // Specific parameters can be added here if needed
    };

    static Status init(const Initializer& initializer) {
        uint32_t i2c_periph = Traits::kBase;
        using SclPinType = typename Config::SclPin;
        using SdaPinType = typename Config::SdaPin;

        // 1. Enable clocks
        rcu_periph_clock_enable(Traits::kRcu); // Enable I2C peripheral clock
        rcu_periph_clock_enable(RCU_AF); // Enable AFIO clock needed for alternate functions
        rcu_periph_clock_enable(RCU_GPIOB);

        // 2. Configure GPIO pins using the Gpio class init method
        SclPinType::init(GpioMode::kAltOpenDrain, GpioPull::kNone, GpioSpeed::kHigh); // Use Alt Open Drain for I2C SCL
        SdaPinType::init(GpioMode::kAltOpenDrain, GpioPull::kNone, GpioSpeed::kHigh); // Use Alt Open Drain for I2C SDA

        // 3. Reset I2C peripheral (using HAL function)
        i2c_deinit(i2c_periph);

        // 4. Configure SMBus mode
        if(initializer.smbus_mode() == I2cMode::kSmbusMode) {
            Traits::smben.set();
            Traits::smbsel.set();
        } else {
            Traits::smben.clear();
            Traits::smbsel.clear();
        }

        // 4. Configure I2C Parameters
        uint32_t clkspeed = 0;
        switch (initializer.speed()) {
            case I2cSpeed::Standard: clkspeed = 100000; break;
            case I2cSpeed::Fast: clkspeed = 400000; break;
            case I2cSpeed::FastPlus: clkspeed = 1000000; break;
        }
        uint32_t dutycyc = (initializer.duty_cycle() == I2cDutyCycle::Duty2) ? I2C_DTCY_2 : I2C_DTCY_16_9;
        i2c_clock_config(i2c_periph, clkspeed, dutycyc); // Use HAL function for complex clock setup

        uint32_t addr_format = (initializer.addr_mode() == I2cAddressingMode::k7bit) ? I2C_ADDFORMAT_7BITS : I2C_ADDFORMAT_10BITS;
        i2c_mode_addr_config(i2c_periph, I2C_I2CMODE_ENABLE, addr_format, 0x00); // Slave address 0 for master mode

        // 5. Configure DMA
        if (initializer.dma_mode() == I2cDmaMode::kEnable) { Traits::dmaon.set(); } 
        else { Traits::dmaon.clear(); }

        // 6. Configure Interrupts
        if (initializer.event_int_mode() == I2cEventInterruptMode::kEnable) {
            nvic_irq_enable(Traits::kEvIrqn, Config::kPreemptPriority, Config::kSubPriority);
            Traits::evie.set();
            Traits::bufie.set();
        } else {
            Traits::evie.clear();
            Traits::bufie.clear();
            nvic_irq_disable(Traits::kEvIrqn);
        }

        if (initializer.error_int_mode() == I2cErrorInterruptMode::kEnable) {
            nvic_irq_enable(Traits::kErIrqn, Config::kPreemptPriority, Config::kSubPriority);
            Traits::errie.set();
        } else {
            Traits::errie.clear();
            nvic_irq_disable(Traits::kErIrqn);
        }

        // 7. Enable I2C
        Traits::i2cen.set();

        // 8. Ack setup
        ack_enable();

        return Status::kOk;
    }

    static Status deinit() {
        uint32_t i2c_periph = Traits::kBase;
        disable();
        i2c_deinit(i2c_periph);
        // Optionally disable clocks and NVIC
        nvic_irq_disable(Traits::kEvIrqn);
        nvic_irq_disable(Traits::kErIrqn);
        rcu_periph_clock_disable(Traits::kRcu);
        rcu_periph_clock_disable(Traits::kRemapRcu);
        return Status::kOk;
    }

    // Timeout value for blocking operations (now in milliseconds)
    static constexpr uint32_t kDefaultTimeout = 100; // e.g., 100ms default timeout

    static I2cStatus write(uint16_t device_addr, const uint8_t* data, size_t length, bool stop = true, uint32_t timeout_ms = kDefaultTimeout) {
        
        if (wait_bus_idle(timeout_ms) != I2cStatus::Ok) return I2cStatus::Busy;

        start_on_bus();
        if (wait_start_sent(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout;

        i2c_master_addressing(Traits::kBase, device_addr, I2C_TRANSMITTER);
        if (wait_address_sent(timeout_ms) != I2cStatus::Ok) {
            if(Traits::aerr) { Traits::aerr = 0; stop_on_bus(); wait_stop_cleared(timeout_ms);} // Use timeout_ms
            return I2cStatus::NackAddr;
        }

        for (size_t i = 0; i < length; ++i) {
            send_byte(data[i]);
            if (wait_tbe(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms
        }

        if (stop) {
            if (wait_btc(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms
            stop_on_bus();
            if (wait_stop_cleared(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout; // Ensure stop completes // Use timeout_ms
        } else {
             // If not stopping, ensure last byte is transmitted (TBE doesn't guarantee shift out)
             if (wait_btc(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms
        }

        return I2cStatus::Ok;
    }

    static I2cStatus read(uint16_t device_addr, uint8_t* data, size_t length, uint32_t timeout_ms = kDefaultTimeout) {

        if (length == 0) return I2cStatus::Ok;
        if (wait_bus_idle(timeout_ms) != I2cStatus::Ok) return I2cStatus::Busy;

        ack_enable(); 
        start_on_bus();
        if (wait_start_sent(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout;

        i2c_master_addressing(Traits::kBase, device_addr, I2C_RECEIVER);
        if (wait_address_sent(timeout_ms) != I2cStatus::Ok) {
             if(Traits::aerr) { Traits::aerr = 0; stop_on_bus(); wait_stop_cleared(timeout_ms);} // Use timeout_ms
            return I2cStatus::NackAddr;
        }

        for (size_t i = 0; i < length; ++i) {
            if (i == length - 1) {
                ack_disable(); // Disable ACK for the last byte
            }
            if (wait_rbne(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms
            data[i] = receive_byte();
        }

        stop_on_bus(); // Send stop condition
        if (wait_stop_cleared(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout; // Ensure stop completes // Use timeout_ms

        ack_enable(); // Re-enable ACK for future transfers

        return I2cStatus::Ok;
    }

    static I2cStatus write_reg(uint16_t device_addr, uint16_t reg_addr, const uint8_t* data, size_t length, uint32_t timeout_ms = kDefaultTimeout) {
        uint8_t reg_addr_byte = static_cast<uint8_t>(reg_addr); // Assuming 8-bit register address
    
        if (wait_bus_idle(timeout_ms) != I2cStatus::Ok) return I2cStatus::Busy;

        start_on_bus();
        if (wait_start_sent(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout;

        i2c_master_addressing(Traits::kBase, device_addr, I2C_TRANSMITTER);
        if (wait_address_sent(timeout_ms) != I2cStatus::Ok) {
             if(Traits::aerr) { Traits::aerr = 0; stop_on_bus(); wait_stop_cleared(timeout_ms);} // Use timeout_ms
            return I2cStatus::NackAddr;
        }

        send_byte(reg_addr_byte);
        if (wait_tbe(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms

        // Important: Ensure register address byte is transmitted before sending data
        if (wait_btc(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms

        for (size_t i = 0; i < length; ++i) {
            send_byte(data[i]);
            if (wait_tbe(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms
        }

        if (wait_btc(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms
        stop_on_bus();
        if (wait_stop_cleared(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout; // Use timeout_ms

        return I2cStatus::Ok;
    }

    static I2cStatus read_reg(uint16_t device_addr, uint16_t reg_addr, uint8_t* data, size_t length, uint32_t timeout_ms = kDefaultTimeout) {
        uint8_t reg_addr_byte = static_cast<uint8_t>(reg_addr); // Assuming 8-bit register address

        if (length == 0) return I2cStatus::Ok;
        if (wait_bus_idle(timeout_ms) != I2cStatus::Ok) return I2cStatus::Busy;

        // --- Write register address phase ---
        start_on_bus();
        if (wait_start_sent(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout;

        i2c_master_addressing(Traits::kBase, device_addr, I2C_TRANSMITTER);
         if (wait_address_sent(timeout_ms) != I2cStatus::Ok) {
            if(Traits::aerr) { Traits::aerr = 0; stop_on_bus(); wait_stop_cleared(timeout_ms);} // Use timeout_ms
            return I2cStatus::NackAddr;
        }

        send_byte(reg_addr_byte);
        // Wait for transmission complete before repeated start
        if (wait_btc(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms

        // --- Read data phase ---
        ack_enable(); // Enable ACK for reading data
        start_on_bus(); // Repeated start
        if (wait_start_sent(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout;

        i2c_master_addressing(Traits::kBase, device_addr, I2C_RECEIVER);
        if (wait_address_sent(timeout_ms) != I2cStatus::Ok) {
             if(Traits::aerr) { Traits::aerr = 0; stop_on_bus(); wait_stop_cleared(timeout_ms);} // Use timeout_ms
            return I2cStatus::NackAddr;
        }

        for (size_t i = 0; i < length; ++i) {
            if (i == length - 1) {
                ack_disable(); // Disable ACK for the last byte
            }
            if (wait_rbne(timeout_ms) != I2cStatus::Ok) { stop_on_bus(); wait_stop_cleared(timeout_ms); return I2cStatus::Timeout; } // Use timeout_ms
            data[i] = receive_byte();
        }

        stop_on_bus(); // Send stop condition
        if (wait_stop_cleared(timeout_ms) != I2cStatus::Ok) return I2cStatus::Timeout; // Ensure stop completes // Use timeout_ms

        ack_enable(); // Re-enable ACK

        return I2cStatus::Ok;
    }

    static I2cStatus write_reg_interrupt(uint16_t device_addr, uint8_t reg_addr, const uint8_t* data, size_t length, uint32_t timeout_ms = kDefaultTimeout) {
        slave_device_addr = device_addr;
        i2c_tx_reg_addr = reg_addr;
        i2c_tx_buffer = data;
        i2c_tx_nBytes = length;
        Traits::i2cen.set();
        while(i2c_tx_nBytes > 0);
        return I2cStatus::Ok;
    }

    static I2cStatus read_reg_interrupt(uint16_t device_addr, uint8_t reg_addr, uint8_t* data, size_t length, uint32_t timeout_ms = kDefaultTimeout) {
        slave_device_addr = device_addr;
        i2c_rx_reg_addr = reg_addr;
        i2c_rx_buffer = data;    
        i2c_rx_nBytes = length;

        if(2 == i2c_rx_nBytes) {
            /* send ACK for the next byte */
            i2c_ackpos_config(Traits::kBase, I2C_ACKPOS_NEXT);
        }
        /* the master waits until the I2C bus is idle */
        while(Traits::i2cbsy);
        /* the master sends a start condition to I2C bus */
        start_on_bus(); 
        while(i2c_rx_nBytes > 0);
        return I2cStatus::Ok;
    }

    static void irq_handler() {
        // 写中断处理函数
        if(i2c_tx_buffer != nullptr) {
            if(Traits::sbsend) {
                /* send slave address */
                i2c_master_addressing(Traits::kBase, slave_device_addr, I2C_TRANSMITTER);
            } else if(Traits::addsend) {
                /*clear ADDSEND bit */
                Traits::addsend = 0;
            }else if(Traits::tbe) {
                if(i2c_tx_reg_addr != 0) {
                    send_byte(i2c_tx_reg_addr);
                    i2c_tx_reg_addr = 0;
                } else if(i2c_tx_reg_addr == 0){
                    if(i2c_tx_nBytes > 0) {
                        /* the master sends a data byte */
                        send_byte(*i2c_tx_buffer++);
                        i2c_tx_nBytes = i2c_tx_nBytes - 1;
                    } else {
                        /* the master sends a stop condition to I2C bus */
                        stop_on_bus();
                        /* disable the I2C0 interrupt */
                        Traits::evie.clear();
                        Traits::bufie.clear();
                    }
                }
            }
        }
        // 读中断处理函数
        if(i2c_rx_buffer != nullptr) {
            if(Traits::sbsend) {
                /* the master sends slave address */
                i2c_master_addressing(Traits::kBase, slave_device_addr, I2C_RECEIVER);
            } else if(Traits::addsend) {
                if((1 == i2c_rx_nBytes) || (2 == i2c_rx_nBytes)) {
                    /* clear the ACKEN before the ADDSEND is cleared */
                    i2c_ack_config(Traits::kBase, I2C_ACK_DISABLE);
                    /* clear the ADDSEND bit */
                    Traits::addsend = 0;
                } else {
                    /* clear the ADDSEND bit */
                    Traits::addsend = 0;
                }
            } else if(Traits::tbe) {
                if(i2c_rx_reg_addr != 0) {
                    *i2c_rx_buffer++ = receive_byte();
                    i2c_rx_reg_addr = 0;
                }
            }  else if(Traits::rbne) {
                if(i2c_rx_nBytes > 0) {
                    if(3 == i2c_rx_nBytes) {
                        /* wait until the second last data byte is received into the shift register */
                        while(!Traits::btc);
                        /* send a NACK for the last data byte */
                        i2c_ack_config(Traits::kBase, I2C_ACK_DISABLE);
                    }
                    /* read a data byte from I2C_DATA*/
                    *i2c_rx_buffer++ = receive_byte();
                    i2c_rx_nBytes = i2c_rx_nBytes - 1;
                    if(0 == i2c_rx_nBytes) {
                        /* send a stop condition */
                        stop_on_bus();
                        i2c_ack_config(Traits::kBase, I2C_ACK_ENABLE);
                        i2c_ackpos_config(Traits::kBase, I2C_ACKPOS_CURRENT);
                        /* disable the I2C0 interrupt */
                        Traits::evie.clear();
                        Traits::bufie.clear();
                        if(i2c_event_callback_) {
                            i2c_event_callback_();
                        }
                    }
                }
            }
        }
    }

    static void reset() {
        Traits::sreset.set();
        __NOP();__NOP();__NOP(); // Short delay
        Traits::sreset.clear();
    }

    static bool isBusy() {
        return Traits::i2cbsy.is_set();
    }

    static I2cStatus getLastError() {
        if(Traits::berr) return I2cStatus::Error;
        if(Traits::lostarb) return I2cStatus::ArbitrationLost;
        if(Traits::aerr) return I2cStatus::NackAddr;
        if(Traits::ouerr) return I2cStatus::Error;
        return I2cStatus::Ok;
    }

    static void clearError() {
        Traits::berr = 0;
        Traits::lostarb = 0;
        Traits::aerr = 0;
        Traits::ouerr = 0;
    }

    static void set_event_callback(I2cEventCallback callback) {
        i2c_event_callback_ = callback;
    }


private:

};

// 简化实例定义
template<size_t Instance>
using I2c = ::hcl::I2c<I2cImpl<Instance>>;

} // namespace gd32f3
} // namespace hcl

#endif // HCL_I2C_IMPL_H