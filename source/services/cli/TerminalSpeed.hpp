#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/FocInteractor.hpp"
#include "source/services/cli/TerminalBase.hpp"

namespace services
{
    class TerminalFocSpeedInteractor
        : public TerminalFocBaseInteractor
    {
    public:
        TerminalFocSpeedInteractor(services::TerminalWithStorage& terminal, FocInteractor& foc, FocSpeedInteractor& focInteractor);

    private:
        StatusWithMessage SetSpeedPid(const infra::BoundedConstString& param);
        StatusWithMessage SetSpeed(const infra::BoundedConstString& param);

    private:
        FocSpeedInteractor& foc;
    };
}
