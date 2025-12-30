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

    void MotorIdentificationWithAlignment::GetInductance(const InductanceConfig& config, const infra::Function<void(std::optional<foc::Henry>)>& onDone)
    {
        inductanceConfig = config;
        onInductanceDone = onDone;
        currentSampleIndex = 0;
        firstCurrent = 0.0f;
        secondCurrent = 0.0f;

        constexpr uint8_t neutralDuty = 50;
        auto voltageOffset = static_cast<uint8_t>(inductanceConfig.testVoltagePercent.Value() * 50 / 100);

        driver.ThreePhasePwmOutput(foc::PhasePwmDutyCycles{
            hal::Percent{ static_cast<uint8_t>(neutralDuty + voltageOffset) },
            hal::Percent{ static_cast<uint8_t>(neutralDuty - voltageOffset) },
            hal::Percent{ neutralDuty } });

        driver.PhaseCurrentsReady(inductanceConfig.samplingFrequency, [this](auto currentPhases)
            {
                if (currentSampleIndex == 0)
                    firstCurrent = currentPhases.a.Value();
                else if (currentSampleIndex == 1)
                {
                    secondCurrent = currentPhases.a.Value();
                    CalculateInductance(currentPhases);
                }
                currentSampleIndex++;
            });
    }

    void MotorIdentificationWithAlignment::CalculateInductance(foc::PhaseCurrents currentPhases)
    {
        driver.Stop();

        float di = secondCurrent - firstCurrent;
        float dt = 1.0f / inductanceConfig.samplingFrequency.Value();
        float diDt = di / dt;

        if (onInductanceDone)
        {
            if (std::abs(di) > inductanceConfig.minCurrentChange.Value())
            {
                auto appliedVoltage = vdc * static_cast<float>(inductanceConfig.testVoltagePercent.Value()) / 100.0f;
                float averageCurrent = (firstCurrent + secondCurrent) / 2.0f;
                float voltageAcrossInductor = appliedVoltage.Value() - (averageCurrent * inductanceConfig.resistance.Value());

                onInductanceDone(std::make_optional<foc::Henry>(voltageAcrossInductor / diDt));
            }
            else
                onInductanceDone(std::nullopt);
        }
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
