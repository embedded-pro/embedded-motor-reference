#pragma once

#include "application/hardware/HardwareFactory.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class TerminalInteractor
    {
    public:
        TerminalInteractor(services::TerminalWithStorage& terminal, application::HardwareFactory& hardware);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage ConfigurePwm(const infra::BoundedConstString& param);
        StatusWithMessage ConfigureAdc(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();
        StatusWithMessage ReadAdcWithSampleTime();
        StatusWithMessage SetPwmDuty(const infra::BoundedConstString& param);

    private:
        services::TerminalWithStorage& terminal;
        application::HardwareFactory& hardware;
    };
}
