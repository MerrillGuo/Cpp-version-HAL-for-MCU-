#include "oscl_common.h"
#include "FreeRTOS.h"

namespace oscl {

void* malloc(size_t size) {
    return pvPortMalloc(size);
}

void free(void* ptr) {
    vPortFree(ptr);
}

} // namespace oscl 