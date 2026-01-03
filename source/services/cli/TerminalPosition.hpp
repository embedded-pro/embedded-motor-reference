#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/TerminalBase.hpp"

namespace services
{
    class TerminalFocPositionInteractor
        : public TerminalFocBaseInteractor
    {
    public:
        TerminalFocPositionInteractor(services::TerminalWithStorage& terminal, foc::Volts vdc, foc::ControllerBase& foc, foc::PositionController& position);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage SetSpeedPid(const infra::BoundedConstString& param);
        StatusWithMessage SetPositionPid(const infra::BoundedConstString& param);
        StatusWithMessage SetPosition(const infra::BoundedConstString& param);

    private:
        foc::Volts vdc;
        foc::PositionController& foc;
    };
}
