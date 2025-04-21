#ifndef APPLICATION_BLDC_LOGIC_HPP
#define APPLICATION_BLDC_LOGIC_HPP

#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/foc/instantiations/SpaceVectorModulatorImpl.hpp"
#include "application/hardware/HardwareFactory.hpp"
#include "application/motors/BLDC/components/FieldOrientedControllerInteractorImpl.hpp"
#include "application/motors/BLDC/components/Terminal.hpp"
#include "application/motors/BLDC/components/TrigonometricImpl.hpp"
#include "application/pid/instantiations/PidImpl.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        TrigonometricFunctions trigonometricFunctions;
        SpaceVectorModulatorImpl spaceVectorModulation;
        PidImpl dPid;
        PidImpl qPid;
        MotorFieldOrientedController::Components components;
        FieldOrientedControllerInteractorImpl focInteractor;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
    };
}

#endif
