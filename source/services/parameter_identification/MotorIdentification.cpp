#include "source/services/parameter_identification/MotorIdentification.hpp"
#include <cmath>
#include <optional>

namespace services
{
    MotorIdentificationWithAlignment::MotorIdentificationWithAlignment(foc::MotorDriver& driver, foc::Encoder& encoder, foc::Volts vdc)
        : driver(driver)
        , encoder(encoder)
        , vdc(vdc)
    {
    }

    void MotorIdentificationWithAlignment::GetResistance(const ResistanceConfig& config, const infra::Function<void(std::optional<foc::Ohm>)>& onDone)
    {
        resistanceConfig = config;
        onResistanceDone = onDone;
        currentSampleIndex = 0;
        accumulatedCurrent = 0.0f;

        constexpr uint8_t neutralDuty = 50;
        auto voltageOffset = static_cast<uint8_t>(resistanceConfig.testVoltagePercent.Value() * 50 / 100);

        driver.ThreePhasePwmOutput(foc::PhasePwmDutyCycles{
            hal::Percent{ static_cast<uint8_t>(neutralDuty + voltageOffset) },
            hal::Percent{ static_cast<uint8_t>(neutralDuty - voltageOffset) },
            hal::Percent{ neutralDuty } });

        driver.PhaseCurrentsReady(hal::Hertz{ 1000 }, [this](auto currentPhases)
            {
                accumulatedCurrent += currentPhases.a.Value();
                currentSampleIndex++;

                if (currentSampleIndex >= resistanceConfig.sampleCount)
                    CalculateResistance(currentPhases);
            });
    }

    void MotorIdentificationWithAlignment::CalculateResistance(foc::PhaseCurrents currentPhases)
    {
        driver.Stop();

        auto averageCurrent = accumulatedCurrent / static_cast<float>(resistanceConfig.sampleCount);
        auto appliedVoltage = vdc * static_cast<float>(resistanceConfig.testVoltagePercent.Value()) / 100.0f;

        if (onResistanceDone)
        {
            if (std::abs(averageCurrent) > resistanceConfig.minCurrent.Value())
                onResistanceDone(std::make_optional<foc::Ohm>(appliedVoltage.Value() / averageCurrent));
            else
                onResistanceDone(std::nullopt);
        }
    }

    void MotorIdentificationWithAlignment::GetInductance(const infra::Function<void(std::optional<foc::Henry>)>& onDone)
    {
        onInductanceDone = onDone;
    }

    void MotorIdentificationWithAlignment::GetNumberOfPolePairs(const infra::Function<void(std::optional<std::size_t>)>& onDone)
    {
        onPolePairsDone = onDone;
    }

    void MotorIdentificationWithAlignment::AlignMotor(std::size_t polePairs, const infra::Function<void(std::optional<foc::Radians>)>& onDone)
    {
        onAlignmentDone = onDone;
    }
}
