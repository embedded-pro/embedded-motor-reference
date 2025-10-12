#ifndef APPLICATION_BLDC_LOGIC_HPP
#define APPLICATION_BLDC_LOGIC_HPP

#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "application/hardware/HardwareFactory.hpp"
#include "application/hardware/MotorFieldOrientedControllerAdapter.hpp"
#include "application/motors/sync_foc_sensored/torque/components/FieldOrientedControllerInteractorImpl.hpp"
#include "application/motors/sync_foc_sensored/torque/components/Terminal.hpp"
#include "application/motors/sync_foc_sensored/torque/instantiations/TrigonometricImpl.hpp"
#include "services/util/DebugLed.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        HardwareAdapter hardwareAdapter;
        TrigonometricFunctions trigonometricFunctions;
        FieldOrientedControllerImpl focImpl{ trigonometricFunctions };
        MotorFieldOrientedControllerImpl motorFocImpl;
        FieldOrientedControllerInteractorImpl focInteractor;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
        services::DebugLed debugLed;
    };
}

#endif
