#ifndef APPLICATION_DC_LOGIC_HPP
#define APPLICATION_DC_LOGIC_HPP

#include "application/dc/logic/MotorControllerImpl.hpp"
#include "application/dc/logic/Terminal.hpp"
#include "application/hardware/HardwareFactory.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class Logic
    {
    public:
        explicit Logic(application::HardwareFactory& hardware);

    private:
        application::HardwareFactory& hardware;
        application::MotorControllerImpl motorController;
        services::TerminalWithStorage::WithMaxSize<10> terminalWithStorage;
        application::TerminalInteractor terminal;
    };
}

#endif
