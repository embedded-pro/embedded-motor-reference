#pragma once

#include "services/util/DebugLed.hpp"
#include "source/application/sync_foc_sensored/instantiations/MotorStateMachine.hpp"
#include "source/foc/implementations/SpeedControllerImpl.hpp"
#include "source/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "source/hardware/HardwareFactory.hpp"
#include "source/hardware/MotorFieldOrientedControllerAdapter.hpp"
#include "source/services/cli/TerminalSpeed.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        HardwareAdapter hardwareAdapter;
        services::DebugLed debugLed;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        MotorStateMachine<
            foc::FieldOrientedControllerSpeedImpl,
            foc::SpeedControllerImpl,
            services::TerminalFocSpeedInteractor>
            motorStateMachine;
    };
}
