#pragma once

#include "services/util/TerminalWithStorage.hpp"
#include "source/services/parameter_identification/MotorIdentification.hpp"

namespace services
{
    class TerminalMotorIdentification
    {
    public:
        TerminalMotorIdentification(services::TerminalWithStorage& terminal, services::Tracer& tracer, MotorIdentification& identification);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        StatusWithMessage EstimateResistanceAndInductance(const infra::BoundedConstString& param);
        StatusWithMessage EstimateNumberOfPolePairs(const infra::BoundedConstString& param);

    private:
        services::TerminalWithStorage& terminal;
        services::Tracer& tracer;
        MotorIdentification& identification;
    };
}
