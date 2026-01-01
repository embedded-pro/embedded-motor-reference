#pragma once

#include "application/sync_foc_sensored/instantiations/MotorStateMachine.hpp"
#include "services/util/DebugLed.hpp"
#include "source/foc/implementations/TorqueControllerImpl.hpp"
#include "source/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "source/foc/instantiations/TrigonometricImpl.hpp"
#include "source/hardware/HardwareFactory.hpp"
#include "source/hardware/MotorFieldOrientedControllerAdapter.hpp"
#include "source/services/cli/TerminalTorque.hpp"

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
            foc::FieldOrientedControllerTorqueImpl,
            foc::TorqueControllerImpl,
            services::TerminalFocTorqueInteractor>
            motorStateMachine;
    };
}
