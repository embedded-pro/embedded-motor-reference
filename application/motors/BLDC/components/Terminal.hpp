#ifndef APPLICATION_BLDC_LOGIC_TERMINAL_HPP
#define APPLICATION_BLDC_LOGIC_TERMINAL_HPP

#include "application/motors/BLDC/components/MotorController.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class TerminalInteractor
    {
    public:
        TerminalInteractor(services::TerminalWithStorage& terminal, application::FocController& focController);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage AutoTune();
        StatusWithMessage SetFocPid(const infra::BoundedConstString& param);
        StatusWithMessage SetSpeedPid(const infra::BoundedConstString& param);
        StatusWithMessage SetSpeed(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();

    private:
        services::TerminalWithStorage& terminal;
        application::FocController& focController;
    };
}

#endif
