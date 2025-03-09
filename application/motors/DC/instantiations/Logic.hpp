#ifndef APPLICATION_DC_LOGIC_HPP
#define APPLICATION_DC_LOGIC_HPP

#include "application/hardware/HardwareFactory.hpp"
#include "application/motors/DC/logic/MotorControllerImpl.hpp"
#include "application/motors/DC/logic/Terminal.hpp"
#include "application/pid/instantiations/PidImpl.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        application::PidImpl pid;
        application::MotorControllerImpl motorController;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
    };
}

#endif
