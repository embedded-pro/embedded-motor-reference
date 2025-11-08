#pragma once

#include "application/foc/implementations/TorqueControllerImpl.hpp"
#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "application/foc/instantiations/TrigonometricImpl.hpp"
#include "application/hardware/HardwareFactory.hpp"
#include "application/hardware/MotorFieldOrientedControllerAdapter.hpp"
#include "application/motors/sync_foc_sensored/torque/components/FieldOrientedControllerInteractorImpl.hpp"
#include "application/motors/sync_foc_sensored/torque/components/Terminal.hpp"
#include "services/util/DebugLed.hpp"

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
        FieldOrientedControllerInteractorImpl focInteractor;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
        services::DebugLed debugLed;
    };
}
