#include "board_teensy41.h"

#include <stdarg.h>
#include <stdio.h>

namespace board {

#if defined(__GNUC__)
__attribute__((format(printf, 4, 5)))
#endif
static void put(char* out, size_t cap, size_t& n, const char* fmt, ...) {
    if (n + 1 >= cap) return;
    va_list a;
    va_start(a, fmt);
    int k = vsnprintf(out + n, cap - n, fmt, a);
    va_end(a);
    if (k > 0) n += (size_t)k < cap - n ? (size_t)k : cap - n - 1;
}

size_t board_report(char* out, size_t cap) {
    size_t n = 0;
    put(out, cap, n, "| Pin | Signal |\n| --- | --- |\n");
    for (uint8_t p = 0; p < N_PINS; ++p)
        for (const auto& u : PINS)
            if (u.pin == p) put(out, cap, n, "| %u | %s |\n", (unsigned)p, u.what);
    put(out, cap, n, "\n%u of %u header pins used, %u spare:", (unsigned)(HEADER_PINS - n_free_header()),
        (unsigned)HEADER_PINS, (unsigned)n_free_header());
    for (uint8_t p = 0; p < HEADER_PINS; ++p) {
        bool used = false;
        for (const auto& u : PINS) used = used || u.pin == p;
        if (!used) put(out, cap, n, " %u", (unsigned)p);
    }
    put(out, cap, n, "\n");
    return n;
}

}  // namespace board
