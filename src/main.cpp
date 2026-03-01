#define PW_LOG_MODULE_NAME "MAIN"
#include "pw_log/log.h"

#include <modm/board.hpp>

using namespace Board;

int main()
{
    Board::initialize();

    PW_LOG_INFO("STM32F429I-DISCO blinky starting");
    PW_LOG_INFO("System clock: %lu Hz", SystemClock::Frequency);

    uint32_t counter = 0;

    while (true)
    {
        LedGreen::toggle();
        modm::delay(500ms);

        LedRed::toggle();
        modm::delay(500ms);

        PW_LOG_INFO("loop %lu", counter++);
    }
}
