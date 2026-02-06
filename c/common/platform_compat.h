/**
 * @file platform_compat.h
 * @brief Cross-platform compatibility layer for Linux/macOS/Windows
 *
 * This header provides platform-independent wrappers for:
 * - Sleep functions (usleep, sleep)
 * - Signal handling
 * - Time functions
 * - Stack trace (debug)
 */

#ifndef PLATFORM_COMPAT_H
#define PLATFORM_COMPAT_H

#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Platform Detection
/****************************************************************************/

#if defined(_WIN32) || defined(_WIN64)
    #define PLATFORM_WINDOWS 1
    #define PLATFORM_UNIX 0
#elif defined(__APPLE__) || defined(__MACH__)
    #define PLATFORM_MACOS 1
    #define PLATFORM_UNIX 1
    #define PLATFORM_WINDOWS 0
#elif defined(__linux__)
    #define PLATFORM_LINUX 1
    #define PLATFORM_UNIX 1
    #define PLATFORM_WINDOWS 0
#else
    #define PLATFORM_UNIX 1
    #define PLATFORM_WINDOWS 0
#endif

/****************************************************************************/
// Platform-specific includes
/****************************************************************************/

#if PLATFORM_WINDOWS
    #include <windows.h>
    #include <signal.h>  // MinGW has signal.h
    // Windows doesn't have these POSIX types
    typedef unsigned int useconds_t;
#else
    #include <unistd.h>
    #include <signal.h>
    #include <sys/time.h>
    #if PLATFORM_LINUX
        #include <execinfo.h>
    #endif
#endif

/****************************************************************************/
// Math constants (not always defined on Windows)
/****************************************************************************/

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

/****************************************************************************/
// Sleep Functions
/****************************************************************************/

/**
 * @brief Sleep for specified microseconds (cross-platform)
 * @param usec Microseconds to sleep
 */
static inline void compat_sleep_us(useconds_t usec) {
#if PLATFORM_WINDOWS
    // Windows Sleep() takes milliseconds, minimum 1ms
    DWORD ms = (usec + 999) / 1000;
    if (ms == 0) ms = 1;
    Sleep(ms);
#else
    usleep(usec);
#endif
}

/**
 * @brief Sleep for specified milliseconds (cross-platform)
 * @param ms Milliseconds to sleep
 */
static inline void compat_sleep_ms(unsigned int ms) {
#if PLATFORM_WINDOWS
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

/**
 * @brief Sleep for specified seconds (cross-platform)
 * @param sec Seconds to sleep
 */
static inline void compat_sleep_s(unsigned int sec) {
#if PLATFORM_WINDOWS
    Sleep(sec * 1000);
#else
    sleep(sec);
#endif
}

// Compatibility macros for existing code using usleep
#if PLATFORM_WINDOWS
    #define usleep(usec) compat_sleep_us(usec)
#endif

/****************************************************************************/
// Signal Handling
/****************************************************************************/

#if PLATFORM_WINDOWS
    // MinGW defines SIGSEGV and SIGABRT in signal.h, don't redefine
    typedef void (*sighandler_t)(int);
    
    static inline sighandler_t signal_compat(int sig, sighandler_t handler) {
        return signal(sig, handler);
    }
#else
    #define signal_compat signal
#endif

/****************************************************************************/
// Stack Trace (Debug)
/****************************************************************************/

/**
 * @brief Print stack trace for debugging (platform-dependent)
 */
static inline void print_stack_trace(void) {
#if PLATFORM_WINDOWS
    fprintf(stderr, "Stack trace not available on Windows\n");
#elif PLATFORM_LINUX
    void *array[10];
    size_t size = backtrace(array, 10);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
#else
    // macOS
    fprintf(stderr, "Stack trace: use lldb or Instruments for detailed debugging\n");
#endif
}

/****************************************************************************/
// Time Functions
/****************************************************************************/

/**
 * @brief Get current time in milliseconds (cross-platform)
 */
static inline long long get_time_ms(void) {
#if PLATFORM_WINDOWS
    return (long long)GetTickCount64();
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)(tv.tv_sec * 1000LL + tv.tv_usec / 1000);
#endif
}

/**
 * @brief Get current time in microseconds (cross-platform)
 */
static inline long long get_time_us(void) {
#if PLATFORM_WINDOWS
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return (long long)(count.QuadPart * 1000000LL / freq.QuadPart);
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)(tv.tv_sec * 1000000LL + tv.tv_usec);
#endif
}

#ifdef __cplusplus
}
#endif

#endif // PLATFORM_COMPAT_H
