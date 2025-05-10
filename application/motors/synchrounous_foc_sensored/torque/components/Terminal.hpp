#pragma once

#include "application/motors/synchrounous_foc_sensored/torque/components/FieldOrientedControllerInteractor.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class TerminalInteractor
    {
    public:
        TerminalInteractor(services::TerminalWithStorage& terminal, FieldOrientedControllerInteractor& focInteractor);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage AutoTune();
        StatusWithMessage SetFocPid(const infra::BoundedConstString& param);
        StatusWithMessage SetTorque(const infra::BoundedConstString& param);
        StatusWithMessage Start();
        StatusWithMessage Stop();

    private:
        services::TerminalWithStorage& terminal;
        FieldOrientedControllerInteractor& focInteractor;
    };
}
