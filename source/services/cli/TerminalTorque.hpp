#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/FocInteractor.hpp"
#include "source/services/cli/TerminalBase.hpp"

namespace services
{
    class TerminalFocTorqueInteractor
        : public TerminalFocBaseInteractor
    {
    public:
        TerminalFocTorqueInteractor(services::TerminalWithStorage& terminal, FocInteractor& foc, FocTorqueInteractor& focInteractor);

    private:
        StatusWithMessage SetTorque(const infra::BoundedConstString& param);

    private:
        FocTorqueInteractor& foc;
    };
}
