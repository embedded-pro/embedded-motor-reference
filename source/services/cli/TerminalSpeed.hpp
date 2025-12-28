#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/FocInteractor.hpp"

namespace services
{
    class TerminalFocSpeedInteractor
    {
    public:
        TerminalFocSpeedInteractor(services::TerminalWithStorage& terminal, FocSpeedInteractor& focInteractor);

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
        FocSpeedInteractor& foc;
    };
}
