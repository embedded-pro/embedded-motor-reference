#include "source/services/parameter_identification/TerminalMotorIdentification.hpp"

namespace services
{
    TerminalMotorIdentification::TerminalMotorIdentification(services::TerminalWithStorage& terminal, services::Tracer& tracer, MotorIdentification& identification)
        : terminal(terminal)
        , tracer(tracer)
        , identification(identification)
    {
        terminal.AddCommand({ { "estimate_resistance", "estr", "Estimate motor resistance. estr <voltage (V)> <duration (s)>. Ex: estr 1.0 0.5" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(EstimateResistance(params));
            } });

        terminal.AddCommand({ { "estimate_inductance", "estl", "Estimate motor inductance. estl <voltage (V)> <duration (s)>. Ex: estl 1.0 0.5" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(EstimateInductance(params));
            } });

        terminal.AddCommand({ { "estimate_pole_pairs", "estpp", "Estimate number of pole pairs. estpp <electrical speed (rad/s)> <mechanical speed (rad/s)>. Ex: estpp 100.0 50.0" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(EstimateNumberOfPolePairs(params));
            } });
    }

    TerminalMotorIdentification::StatusWithMessage TerminalMotorIdentification::EstimateResistance(const infra::BoundedConstString& param)
    {
        MotorIdentification::ResistanceConfig config;

        identification.GetResistance(config, [this](auto resistance)
            {
                if (!resistance.has_value())
                    tracer.Trace() << "Resistance estimation failed.";
                else
                    tracer.Trace() << "Estimated Resistance: " << resistance->Value() << " Ohms";
            });
        return TerminalMotorIdentification::StatusWithMessage();
    }

    TerminalMotorIdentification::StatusWithMessage TerminalMotorIdentification::EstimateInductance(const infra::BoundedConstString& param)
    {
        MotorIdentification::InductanceConfig config;

        identification.GetInductance(config, [this](auto inductance)
            {
                if (!inductance.has_value())
                    tracer.Trace() << "Inductance estimation failed.";
                else
                    tracer.Trace() << "Estimated Inductance: " << inductance->Value() << " H";
            });

        return TerminalMotorIdentification::StatusWithMessage();
    }

    TerminalMotorIdentification::StatusWithMessage TerminalMotorIdentification::EstimateNumberOfPolePairs(const infra::BoundedConstString& param)
    {
        MotorIdentification::PolePairsConfig config;

        identification.GetNumberOfPolePairs(config, [this](auto polePairs)
            {
                if (!polePairs.has_value())
                    tracer.Trace() << "Pole pairs estimation failed.";
                else
                    tracer.Trace() << "Estimated Pole Pairs: " << *polePairs;
            });

        return TerminalMotorIdentification::StatusWithMessage();
    }
}
