#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <cstdint>
#include <cstring>
#include <span>

namespace hcl {

// 轻量级的min函数实现
template<typename T>
static inline T Min(T a, T b) {
    return (a < b) ? a : b;
}

/**
 * @brief A lightweight ring buffer implementation for embedded systems.
 * 
 * @note This ring buffer is NOT thread-safe. When used in an interrupt context
 * or multi-tasking environment, the caller must ensure proper synchronization:
 * - Disable interrupts when accessing the buffer from a task if it's also accessed from ISR
 * - Use mutex/critical section when accessing from multiple tasks
 * 
 * Example usage in ISR context:
 * ```cpp
 * // In task:
 * uint32_t primask = __get_PRIMASK();
 * __disable_irq();
 * buffer.Push(data);
 * __set_PRIMASK(primask);
 * 
 * // In ISR:
 * buffer.Pop(data);
 * ```
 */
template<typename T, size_t SIZE>
class RingBuffer {
public:
    RingBuffer() : head_(0), tail_(0), size_(0) {}

    bool push(const T& data) {
        if (is_full()) {
            return false;
        }
        buffer_[tail_] = data;
        tail_ = (tail_ + 1) % SIZE;
        size_++;
        return true;
    }

    bool push(std::span<const T> data) {
        if (available() < data.size()) {
            return false;
        }
        
        size_t first_chunk = Min(data.size(), SIZE - tail_);
        memcpy(&buffer_[tail_], data.data(), first_chunk * sizeof(T));
        
        if (first_chunk < data.size()) {
            memcpy(buffer_, data.data() + first_chunk, 
                  (data.size() - first_chunk) * sizeof(T));
        }
        
        tail_ = (tail_ + data.size()) % SIZE;
        size_ += data.size();
        return true;
    }

    bool fake_push(size_t length) {
        if (available() < length) {
            return false;
        }
        
        tail_ = (tail_ + length) % SIZE;
        size_ += length;
        return true;
    }

    bool pop(T& data) {
        if (empty()) {
            return false;
        }
        data = buffer_[head_];
        head_ = (head_ + 1) % SIZE;
        size_--;
        return true;
    }

    size_t pop(std::span<T> data) {
        size_t length = Min(data.size(), size_);
        if (length == 0) {
            return 0;
        }

        size_t first_chunk = Min(length, SIZE - head_);
        memcpy(data.data(), &buffer_[head_], first_chunk * sizeof(T));
        
        if (first_chunk < length) {
            memcpy(data.data() + first_chunk, buffer_, 
                  (length - first_chunk) * sizeof(T));
        }
        
        head_ = (head_ + length) % SIZE;
        size_ -= length;
        return length;
    }

    size_t peek(std::span<T> data) const {
        size_t length = Min(data.size(), size_);
        if (length == 0) {
            return 0;
        }

        size_t first_chunk = Min(length, SIZE - head_);
        memcpy(data.data(), &buffer_[head_], first_chunk * sizeof(T));
        
        if (first_chunk < length) {
            memcpy(data.data() + first_chunk, buffer_, 
                  (length - first_chunk) * sizeof(T));
        }
        
        return length;
    }

    size_t tail_position() const {
        return tail_;
    }

    // 获取内部缓冲区指针，用于DMA操作
    T* buffer() {
        return buffer_;
    }

    bool empty() const { return size_ == 0; }
    bool is_full() const { return size_ == SIZE; }
    size_t used() const { return size_; }
    size_t capacity() const { return SIZE; }
    size_t available() const { return SIZE - size_; }

    void clear() {
        head_ = tail_ = size_ = 0;
    }

private:
    T buffer_[SIZE];
    size_t head_;
    size_t tail_;
    size_t size_;
};

} // namespace hcl

#endif // RING_BUFFER_H 