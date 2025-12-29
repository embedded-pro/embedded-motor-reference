#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/FocInteractor.hpp"

namespace services
{
    class TerminalFocBaseInteractor
    {
    protected:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        TerminalFocBaseInteractor(services::TerminalWithStorage& terminal, FocInteractor& focInteractor);

        services::TerminalWithStorage& Terminal();

    private:
        StatusWithMessage AutoTune();
        StatusWithMessage SetFocPid(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();

    private:
        services::TerminalWithStorage& terminal;
        FocInteractor& foc;
    };
}
