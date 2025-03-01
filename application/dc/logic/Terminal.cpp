#include "application/dc/logic/Terminal.hpp"

namespace application
{
    TerminalInteractor::TerminalInteractor(services::TerminalCommands& terminal, services::Tracer& tracer, application::MotorController& motorController)
        : terminal(terminal)
        , tracer(tracer)
        , motorController(motorController)
    {
    }

    void TerminalInteractor::AutoTune()
    {
    }

    void TerminalInteractor::SetKpKiKd(const infra::BoundedConstString& param)
    {
    }

    void TerminalInteractor::SetSpeed(const infra::BoundedConstString& param)
    {
    }

    void TerminalInteractor::Start()
    {
    }

    void TerminalInteractor::Stop()
    {
    }
}
