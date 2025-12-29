#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/FocInteractor.hpp"
#include "source/services/cli/TerminalBase.hpp"

namespace services
{
    class TerminalFocPositionInteractor
        : public TerminalFocBaseInteractor
    {
    public:
        TerminalFocPositionInteractor(services::TerminalWithStorage& terminal, FocInteractor& foc, FocPositionInteractor& focInteractor);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage SetSpeedPid(const infra::BoundedConstString& param);
        StatusWithMessage SetPositionPid(const infra::BoundedConstString& param);
        StatusWithMessage SetPosition(const infra::BoundedConstString& param);

    private:
        FocPositionInteractor& foc;
    };
}
