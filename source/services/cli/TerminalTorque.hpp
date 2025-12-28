#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/FocInteractor.hpp"

namespace services
{
    class TerminalFocTorqueInteractor
    {
    public:
        TerminalFocTorqueInteractor(services::TerminalWithStorage& terminal, FocTorqueInteractor& focInteractor);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage AutoTune();
        StatusWithMessage SetFocPid(const infra::BoundedConstString& param);
        StatusWithMessage SetTorque(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();

    private:
        services::TerminalWithStorage& terminal;
        FocTorqueInteractor& foc;
    };
}
