#pragma once

#include "services/util/DebugLed.hpp"
#include "source/foc/implementations/SpeedControllerImpl.hpp"
#include "source/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "source/foc/instantiations/TrigonometricImpl.hpp"
#include "source/hardware/HardwareFactory.hpp"
#include "source/hardware/MotorFieldOrientedControllerAdapter.hpp"
#include "source/services/cli/FocSpeedInteractorImpl.hpp"
#include "source/services/cli/TerminalSpeed.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        HardwareAdapter hardwareAdapter;
        foc::TrigonometricFunctions trigonometricFunctions;
        foc::FieldOrientedControllerSpeedImpl focImpl;
        foc::SpeedControllerImpl motorFocImpl;
        services::FocSpeedInteractorImpl focInteractor;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        services::TerminalFocSpeedInteractor terminal;
        services::DebugLed debugLed;
    };
}
