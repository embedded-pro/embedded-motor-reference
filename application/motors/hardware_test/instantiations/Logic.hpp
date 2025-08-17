#ifndef APPLICATION_BLDC_LOGIC_HPP
#define APPLICATION_BLDC_LOGIC_HPP

#include "application/hardware/HardwareFactory.hpp"
#include "application/motors/hardware_test/components/Terminal.hpp"
#include "services/util/DebugLed.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
        services::DebugLed debugLed;
    };
}

#endif
