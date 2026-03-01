// pw_log backend implementation for modm.
//
// Formats the log message into a stack buffer using vsnprintf, then writes
// it through modm's IOStream logger which is connected to USART1 (ST-LINK
// virtual COM port on the STM32F429I-DISCO).

#include "pw_log_backend/log_backend.h"

#include <cstdarg>
#include <cstdio>

#include <modm/debug/logger.hpp>
#include "pw_log/levels.h"

extern "C" void pw_log_modm_log(int level,
                                const char* module,
                                const char* /*file*/,
                                int /*line*/,
                                const char* message,
                                ...)
{
    // Format the user message into a temporary buffer.
    char buf[256];
    va_list args;
    va_start(args, message);
    vsnprintf(buf, sizeof(buf), message, args);
    va_end(args);

    // Pick the level tag and the matching modm log stream.
    const char* tag;
    switch (level) {
        case PW_LOG_LEVEL_DEBUG:    tag = "DBG"; break;
        case PW_LOG_LEVEL_INFO:     tag = "INF"; break;
        case PW_LOG_LEVEL_WARN:     tag = "WRN"; break;
        case PW_LOG_LEVEL_ERROR:    tag = "ERR"; break;
        case PW_LOG_LEVEL_CRITICAL: tag = "FTL"; break;
        default:                    tag = "???"; break;
    }

    // Write "[LEVEL MODULE] message" through the appropriate modm logger.
    // modm routes these to the USART1 buffer set up by Board::initialize().
    switch (level) {
        case PW_LOG_LEVEL_DEBUG:
            MODM_LOG_DEBUG   << "[" << tag << " " << module << "] " << buf << modm::endl;
            break;
        case PW_LOG_LEVEL_INFO:
            MODM_LOG_INFO    << "[" << tag << " " << module << "] " << buf << modm::endl;
            break;
        case PW_LOG_LEVEL_WARN:
            MODM_LOG_WARNING << "[" << tag << " " << module << "] " << buf << modm::endl;
            break;
        default:
            MODM_LOG_ERROR   << "[" << tag << " " << module << "] " << buf << modm::endl;
            break;
    }
}
