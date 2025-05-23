#ifndef APPLICATION_BLDC_LOGIC_HPP
#define APPLICATION_BLDC_LOGIC_HPP

#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "application/hardware/HardwareFactory.hpp"
#include "application/motors/synchrounous_foc_sensored/torque/components/FieldOrientedControllerInteractorImpl.hpp"
#include "application/motors/synchrounous_foc_sensored/torque/components/Terminal.hpp"
#include "application/motors/synchrounous_foc_sensored/torque/components/TrigonometricImpl.hpp"
#include "services/util/DebugLed.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
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
