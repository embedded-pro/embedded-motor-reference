#ifndef APPLICATION_BLDC_LOGIC_HPP
#define APPLICATION_BLDC_LOGIC_HPP

#include "application/foc/instantiations/SpaceVectorModulatorImpl.hpp"
#include "application/hardware/HardwareFactory.hpp"
#include "application/motors/BLDC/components/MotorControllerImpl.hpp"
#include "application/motors/BLDC/components/Terminal.hpp"
#include "application/motors/BLDC/components/TrigonometricImpl.hpp"
#include <chrono>

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
        FocWithTimer::Components components;
        FocControllerImpl::Input input;
        FocControllerImpl focController;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
    };
}

#endif
