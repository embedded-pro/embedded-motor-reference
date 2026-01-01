#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/foc/interfaces/Controller.hpp"

namespace services
{
    class TerminalFocBaseInteractor
    {
    protected:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        TerminalFocBaseInteractor(services::TerminalWithStorage& terminal, foc::Volts vdc, foc::ControllerBase& foc);

        services::TerminalWithStorage& Terminal();

    private:
        StatusWithMessage SetFocPid(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();

    private:
        services::TerminalWithStorage& terminal;
        foc::Volts vdc;
        foc::ControllerBase& foc;
    };
}
