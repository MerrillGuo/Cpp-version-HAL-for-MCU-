#ifndef TESTS_OSCL_MOCK_TIME_HPP_
#define TESTS_OSCL_MOCK_TIME_HPP_

#include "oscl_common.h"
#include <cstdint>

namespace oscl {

// Global time offset for tests
inline uint32_t g_time_offset_ms = 0;

/**
 * @brief Advance simulated time by a specified number of milliseconds
 * 
 * This function is used in tests to simulate the passage of time
 * without actually waiting. It affects the return value of get_tick_ms().
 * 
 * @param ms Number of milliseconds to advance
 */
inline void advance_time(uint32_t ms) {
    g_time_offset_ms += ms;
}

/**
 * @brief Reset simulated time to zero
 */
inline void reset_time() {
    g_time_offset_ms = 0;
}

/**
 * @brief Set simulated time to a specific value
 * 
 * @param ms Specific time in milliseconds
 */
inline void set_time(uint32_t ms) {
    g_time_offset_ms = ms;
}

// Forward declaration of our internal function in the .cpp file
uint32_t _real_get_tick_ms();

/**
 * @brief Get system time in milliseconds
 * 
 * This function overrides the real oscl::get_tick_ms() in tests,
 * returning the simulated time for deterministic testing.
 * 
 * @return Current simulated system time in milliseconds
 */
inline uint32_t get_tick_ms() {
    // Return the base system time plus our offset
    return _real_get_tick_ms() + g_time_offset_ms;
}


} // namespace oscl

#endif // TESTS_OSCL_MOCK_TIME_HPP_ 