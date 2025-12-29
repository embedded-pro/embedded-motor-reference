#ifndef APPLICATION_BLDC_LOGIC_HPP
#define APPLICATION_BLDC_LOGIC_HPP

#include "services/util/DebugLed.hpp"
#include "source/application/hardware_test/components/Terminal.hpp"
#include "source/hardware/HardwareFactory.hpp"

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
