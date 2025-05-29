#include "board_config.h"

#include <cstdio>

#ifdef __GNUC__

extern "C" __attribute__((used, visibility("default"))) int _write(int file, char *ptr, int len) {
    (void)file;
    DebugUart::send(std::span{reinterpret_cast<const uint8_t*>(ptr), static_cast<size_t>(len)});
    return len;
}

extern "C" int _close_r(int, void*) { return -1; }
extern "C" int _fstat_r(int, void*) { return -1; }
extern "C" int _getpid_r(void) { return -1; }
extern "C" int _isatty_r(int, void*) { return 0; }
extern "C" int _kill_r(int, int) { return -1; }
extern "C" int _lseek_r(int, int, int) { return -1; }
extern "C" int _read_r(int, void*, unsigned int) { return 0; }

#endif