#pragma once

#include "application/motors/dc/components/MotorPidController.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class TerminalInteractor
    {
    public:
        TerminalInteractor(services::TerminalWithStorage& terminal, MotorPidController& motorController);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage AutoTune();
        StatusWithMessage SetKpKiKd(const infra::BoundedConstString& param);
        StatusWithMessage SetSpeed(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();

    private:
        services::TerminalWithStorage& terminal;
        application::MotorPidController& motorController;
    };
}
