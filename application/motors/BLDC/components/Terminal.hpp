#ifndef APPLICATION_BLDC_LOGIC_TERMINAL_HPP
#define APPLICATION_BLDC_LOGIC_TERMINAL_HPP

#include "application/motors/DC/components/MotorController.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class TerminalInteractor
    {
    public:
        TerminalInteractor(services::TerminalWithStorage& terminal, application::MotorController& motorController);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage AutoTune();
        StatusWithMessage SetKpKiKd(const infra::BoundedConstString& param);
        StatusWithMessage SetSpeed(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();

    private:
        services::TerminalWithStorage& terminal;
        application::MotorController& motorController;
    };
}

#endif
