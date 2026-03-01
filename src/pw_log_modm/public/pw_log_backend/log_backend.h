// pw_log backend that routes through modm's UART logger.
//
// This header satisfies the pw_log facade contract by providing
// the PW_HANDLE_LOG macro that pw_log/log.h requires from its backend.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Implemented in log_backend.cc; uses modm's IOStream to write to USART1.
void pw_log_modm_log(int level,
                     const char* module,
                     const char* file,
                     int line,
                     const char* message,
                     ...) __attribute__((format(printf, 5, 6)));

#ifdef __cplusplus
}
#endif

// Required by pw_log/log.h — routes every enabled log call to our function.
#define PW_HANDLE_LOG(level, module, flags, message, ...) \
    pw_log_modm_log((level), (module), __FILE__, __LINE__, message, ##__VA_ARGS__)
