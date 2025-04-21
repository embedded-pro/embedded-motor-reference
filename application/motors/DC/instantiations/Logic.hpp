#pragma once

#include "application/hardware/HardwareFactory.hpp"
#include "application/motors/DC/components/MotorPidControllerImpl.hpp"
#include "application/motors/DC/components/Terminal.hpp"
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
        application::MotorPidControllerImpl motorController;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
    };
}
