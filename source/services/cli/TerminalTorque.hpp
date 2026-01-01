#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/TerminalBase.hpp"

namespace services
{
    class TerminalFocTorqueInteractor
        : public TerminalFocBaseInteractor
    {
    public:
        TerminalFocTorqueInteractor(services::TerminalWithStorage& terminal, foc::Volts vdc, foc::ControllerBase& foc, foc::TorqueController& torque);

    private:
        StatusWithMessage SetTorque(const infra::BoundedConstString& param);

    private:
        foc::TorqueController& foc;
    };
}
