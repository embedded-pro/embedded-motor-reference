#pragma once

#include "services/util/DebugLed.hpp"
#include "source/foc/implementations/TorqueControllerImpl.hpp"
#include "source/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "source/foc/instantiations/TrigonometricImpl.hpp"
#include "source/hardware/HardwareFactory.hpp"
#include "source/hardware/MotorFieldOrientedControllerAdapter.hpp"
#include "source/services/terminal/FocTorqueInteractorImpl.hpp"
#include "source/services/terminal/TerminalTorque.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        HardwareAdapter hardwareAdapter;
        foc::TrigonometricFunctions trigonometricFunctions;
        foc::FieldOrientedControllerTorqueImpl focImpl{ trigonometricFunctions };
        foc::TorqueControllerImpl motorFocImpl;
        services::FocTorqueInteractorImpl focInteractor;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        services::TerminalFocTorqueInteractor terminal;
        services::DebugLed debugLed;
    };
}
