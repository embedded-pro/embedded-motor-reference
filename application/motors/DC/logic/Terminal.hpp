#ifndef APPLICATION_DC_LOGIC_TERMINAL_HPP
#define APPLICATION_DC_LOGIC_TERMINAL_HPP

#include "application/motors/DC/logic/MotorController.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/Terminal.hpp"

namespace application
{
    class TerminalInteractor
    {
    public:
        TerminalInteractor(services::TerminalCommands& terminal, services::Tracer& tracer, application::MotorController& motorController);

    private:
        void AutoTune();
        void SetKpKiKd(const infra::BoundedConstString& param);
        void SetSpeed(const infra::BoundedConstString& param);
        void Start();
        void Stop();

    private:
        services::TerminalCommands& terminal;
        services::Tracer& tracer;
        application::MotorController& motorController;
    };
}

#endif
