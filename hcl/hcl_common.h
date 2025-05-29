#ifndef HCL_COMMON_H
#define HCL_COMMON_H

#include <cstdint>

namespace hcl {

/**
 * @brief Common status codes for all HCL modules
 */
enum class Status {
    kOk = 0,            ///< 操作成功
    kError,             ///< 通用错误
    kBusy,              ///< 设备忙
    kTimeout,           ///< 操作超时
    kInvalidParameter,  ///< 参数无效
    kInvalidOperation,  ///< 操作无效（如在错误的状态下调用）
    kNotInitialized,    ///< 未初始化
};

} // namespace hcl

#endif // HCL_COMMON_H