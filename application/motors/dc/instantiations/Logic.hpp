#pragma once

#include "application/hardware/HardwareFactory.hpp"
#include "application/hardware/PidAdapter.hpp"
#include "application/motors/dc/components/MotorPidControllerImpl.hpp"
#include "application/motors/dc/components/Terminal.hpp"
#include "numerical/controllers/implementations/PidIncremental.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        application::PidDriverImpl pidDriver;
        controllers::PidIncrementalAsynchronous<float> pid;
        application::MotorPidControllerImpl motorController;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
    };
}
