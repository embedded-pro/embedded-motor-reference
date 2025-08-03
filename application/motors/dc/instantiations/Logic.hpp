#pragma once

#include "application/hardware/HardwareFactory.hpp"
#include "application/motors/dc/components/MotorPidControllerImpl.hpp"
#include "application/motors/dc/components/Terminal.hpp"
#include "application/motors/dc/instantiations/HardwareAdapter.hpp"
#include "application/pid/PidImpl.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        application::HardwareAdapter hardwareAdapter;
        application::PidImpl pid;
        application::MotorPidControllerImpl motorController;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
    };
}
