// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// The tests are compiled with the same Release flags as the library, and Release
// defines NDEBUG, which turns every assert() into a no-op. GENZ_CHECK is always
// live, so a failing test actually fails.
#pragma once

#include <cstdio>
#include <cstdlib>

#define GENZ_CHECK(condition)                                                            \
    do {                                                                                 \
        if (!(condition)) {                                                              \
            std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__,        \
                         #condition);                                                    \
            std::abort();                                                                \
        }                                                                                \
    } while (false)
