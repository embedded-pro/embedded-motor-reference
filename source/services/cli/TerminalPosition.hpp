#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/FocInteractor.hpp"

namespace services
{
    class TerminalFocPositionInteractor
    {
    public:
        TerminalFocPositionInteractor(services::TerminalWithStorage& terminal, FocPositionInteractor& focInteractor);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage AutoTune();
        StatusWithMessage SetFocPid(const infra::BoundedConstString& param);
        StatusWithMessage SetSpeedPid(const infra::BoundedConstString& param);
        StatusWithMessage SetPositionPid(const infra::BoundedConstString& param);
        StatusWithMessage SetPosition(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();

    private:
        services::TerminalWithStorage& terminal;
        FocPositionInteractor& foc;
    };
}
