#ifndef TESTS_HCL_MOCK_UART_HPP_
#define TESTS_HCL_MOCK_UART_HPP_

#include "hcl_uart.h"
#include "ring_buffer.h"
#include "oscl_common.h"

#include <functional>
#include <array>
#include <cstdint>
#include <span>
#include <cstring>

namespace hcl {
namespace mock {

/**
 * @brief Mock UART implementation for testing
 * 
 * This class implements the HCL UART interface for testing purposes.
 * It uses RingBuffer internally to simulate UART behavior.
 * 
 * @tparam Id UART instance ID (arbitrary for mock)
 * @tparam RxBufSize Receive buffer size in bytes
 * @tparam TxBufSize Transmit buffer size in bytes
 */
template <size_t Id, size_t RxBufSize = 128, size_t TxBufSize = 128>
class MockUart {
public:
    // Configure pin types for UartImpl concept
    struct MockPin {};
    using TxPin = MockPin;
    using RxPin = MockPin;
    
    // Configure modes for UartImpl concept
    static constexpr UartMode kRxMode = UartMode::kInterrupt;
    static constexpr UartMode kTxMode = UartMode::kInterrupt;
    static constexpr UartTrigger kRxTrigger = UartTrigger::kNormal;
    static constexpr size_t kRxBufSize = RxBufSize;
    static constexpr size_t kTxBufSize = TxBufSize;
    static constexpr uint8_t kPreemptPriority = 7;
    static constexpr uint8_t kSubPriority = 0;

    /// Configuration storage for initialization settings
    struct UartConfig {
        uint32_t baudrate{115200};
        UartWordLength length{UartWordLength::kBits8};
        UartParity parity{UartParity::kNone};
        UartStopBits stop_bits{UartStopBits::kBits1};
        UartMode mode{UartMode::kInterrupt};
        UartTrigger trigger{UartTrigger::kNormal};
    };

    /**
     * @brief Initialize UART with configuration
     * @param config Configuration settings
     * @return Status of initialization
     */
    template<typename InitializerT>
    static Status init(const UartInitializerBase<InitializerT>& config) {
        // Copy settings from initializer
        config_.baudrate = config.baudrate();
        config_.length = config.length();
        config_.parity = config.parity();
        config_.stop_bits = config.stop_bits();
        
        // Reset state
        rx_buffer_.clear();
        tx_buffer_.clear();
        rx_callback_ = nullptr;
        tx_callback_ = nullptr;
        error_callback_ = nullptr;
        receiving_ = false;
        
        return Status::kOk;
    }
    
    /**
     * @brief UART initializer for method chaining
     */
    class Initializer : public UartInitializerBase<Initializer> {
    public:
        explicit Initializer(UartConfig& config) : config_(config) {}
        
        Initializer& baudrate(uint32_t value) {
            config_.baudrate = value;
            return *this;
        }
        
        Initializer& length(UartWordLength value) {
            config_.length = value;
            return *this;
        }
        
        Initializer& parity(UartParity value) {
            config_.parity = value;
            return *this;
        }
        
        Initializer& stop_bits(UartStopBits value) {
            config_.stop_bits = value;
            return *this;
        }
        
        Initializer& mode(UartMode value) {
            config_.mode = value;
            return *this;
        }
        
        Initializer& trigger(UartTrigger value) {
            config_.trigger = value;
            return *this;
        }
        
    private:
        UartConfig& config_;
    };

    /**
     * @brief Initialize UART
     * @return Initializer object for method chaining
     */
    static Initializer init() {
        // Reset buffers and state
        rx_buffer_.clear();
        tx_buffer_.clear();
        rx_callback_ = nullptr;
        tx_callback_ = nullptr;
        error_callback_ = nullptr;
        receiving_ = false;
        
        return Initializer(config_);
    }
    
    /**
     * @brief Deinitialize UART
     * @return Status of operation
     */
    static Status deinit() {
        // Reset state
        rx_buffer_.clear();
        tx_buffer_.clear();
        rx_callback_ = nullptr;
        tx_callback_ = nullptr;
        error_callback_ = nullptr;
        receiving_ = false;
        return Status::kOk;
    }
    
    /**
     * @brief Send data over UART
     * @param data Data to send
     * @return Status of operation
     */
    static Status send(std::span<const uint8_t> data) {
        // Check if we can fit all data
        if (data.size() > tx_buffer_.available()) {
            return Status::kError;
        }
        
        // Add data to TX buffer
        for (size_t i = 0; i < data.size(); i++) {
            if (!tx_buffer_.push(data[i])) {
                return Status::kError;
            }
        }
        
        // Simulate TX completion callback if registered
        if (tx_callback_) {
            tx_callback_();
        }
        
        return Status::kOk;
    }
    
    /**
     * @brief Read data from UART
     * @param data Buffer to store read data
     * @return Number of bytes read
     */
    static size_t read(std::span<uint8_t> data) {
        size_t bytes_read = 0;
        
        // Read data from RX buffer
        while (!rx_buffer_.empty() && bytes_read < data.size()) {
            uint8_t value;
            if (rx_buffer_.pop(value)) {
                data[bytes_read++] = value;
            }
        }
        
        return bytes_read;
    }
    
    /**
     * @brief Get number of bytes available to read
     * @return Number of bytes in RX buffer
     */
    static size_t rx_size() {
        return rx_buffer_.used();
    }
    
    /**
     * @brief Clear RX buffer
     */
    static void clear_rx() {
        rx_buffer_.clear();
    }
    
    /**
     * @brief Set RX callback
     * @param callback Function to call when data is received
     */
    static void set_rx_callback(UartRxCallback callback) {
        rx_callback_ = callback;
    }
    
    /**
     * @brief Set TX callback
     * @param callback Function to call when data is transmitted
     */
    static void set_tx_callback(UartTxCallback callback) {
        tx_callback_ = callback;
    }
    
    /**
     * @brief Set error callback
     * @param callback Function to call on error
     */
    static void set_error_callback(UartErrorCallback callback) {
        error_callback_ = callback;
    }
    
    /**
     * @brief Start receiving data
     * @param length Maximum number of bytes to receive (0 for unlimited)
     * @return Status of operation
     */
    static Status start_receive(size_t length = 0) {
        (void)length;  // Unused in mock implementation
        receiving_ = true;
        return Status::kOk;
    }
    
    /**
     * @brief Stop receiving data
     * @return Status of operation
     */
    static Status stop_receive() {
        receiving_ = false;
        return Status::kOk;
    }
    
    /**
     * @brief UART IRQ handler (unused in mock)
     */
    static void irq_handler() {
        // Not needed for mock implementation
    }
    
    // Mock-specific methods
    
    /**
     * @brief Simulate receiving data from external source
     * @param data Data to simulate receiving
     * @param len Length of data
     * @return true if data was successfully received
     */
    static bool mock_receive_data(const uint8_t* data, size_t len) {
        if (!receiving_) {
            return false;
        }
        
        // Check if we can fit all data
        if (len > rx_buffer_.available()) {
            // Trigger error callback if registered
            if (error_callback_) {
                error_callback_(UartError::kOverrun);
            }
            return false;
        }

        rx_buffer_.push(std::span<const uint8_t>(data, len));
        
        // Trigger RX callback if registered
        if (rx_callback_) {
            rx_callback_(len);
        }
        
        return true;
    }
    
    /**
     * @brief Simulate error condition
     * @param error Error to simulate
     */
    static void mock_error(UartError error) {
        if (error_callback_) {
            error_callback_(error);
        }
    }
    
    /**
     * @brief Get copy of current configuration
     * @return Current UART configuration
     */
    static UartConfig get_config() {
        return config_;
    }
    
    /**
     * @brief Reset mock UART state
     */
    static void reset() {
        rx_buffer_.clear();
        tx_buffer_.clear();
        rx_callback_ = nullptr;
        tx_callback_ = nullptr;
        error_callback_ = nullptr;
        receiving_ = false;
        config_ = UartConfig{};
    }
    
    /**
     * @brief Check if UART is receiving
     * @return true if UART is in receiving state
     */
    static bool is_receiving() {
        return receiving_;
    }
    
    /**
     * @brief Get data from TX buffer (for testing outgoing data)
     * @param data Buffer to store data
     * @return Number of bytes read
     */
    static size_t read_tx_buffer(std::span<uint8_t> data) {
        size_t bytes_read = 0;
        
        while (!tx_buffer_.empty() && bytes_read < data.size()) {
            uint8_t value;
            if (tx_buffer_.pop(value)) {
                data[bytes_read++] = value;
            }
        }
        
        return bytes_read;
    }
    
    /**
     * @brief Get TX buffer size
     * @return Number of bytes in TX buffer
     */
    static size_t tx_size() {
        return tx_buffer_.used();
    }

private:
    // Static storage for UART state
    static inline UartConfig config_{};
    static inline RingBuffer<uint8_t, RxBufSize> rx_buffer_{};
    static inline RingBuffer<uint8_t, TxBufSize> tx_buffer_{};
    static inline UartRxCallback rx_callback_{};
    static inline UartTxCallback tx_callback_{};
    static inline UartErrorCallback error_callback_{};
    static inline bool receiving_{false};
};

} // namespace mock
} // namespace hcl

#endif // TESTS_HCL_MOCK_UART_HPP_ 