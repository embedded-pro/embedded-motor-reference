#include "source/services/parameter_identification/MotorIdentificationImpl.hpp"
#include <cmath>
#include <numbers>

namespace
{
    constexpr float twoPi = 2.0f * std::numbers::pi_v<float>;
    constexpr std::size_t stepsPerRevolution = 12;

    foc::PhasePwmDutyCycles NormalizedDutyCycles(foc::ThreePhase voltages)
    {
        auto offset = 50.0f;
        auto dutyA = static_cast<uint8_t>(std::clamp(offset + voltages.a * 50.0f, 0.0f, 100.0f));
        auto dutyB = static_cast<uint8_t>(std::clamp(offset + voltages.b * 50.0f, 0.0f, 100.0f));
        auto dutyC = static_cast<uint8_t>(std::clamp(offset + voltages.c * 50.0f, 0.0f, 100.0f));
        return foc::PhasePwmDutyCycles{ hal::Percent{ dutyA }, hal::Percent{ dutyB }, hal::Percent{ dutyC } };
    }
}

namespace services
{
    MotorIdentificationImpl::MotorIdentificationImpl(foc::MotorDriver& driver, foc::Encoder& encoder, foc::Volts vdc)
        : driver(driver)
        , encoder(encoder)
        , vdc(vdc)
    {
    }

    void MotorIdentificationImpl::GetResistance(const ResistanceConfig& config, const infra::Function<void(std::optional<foc::Ohm>)>& onDone)
    {
        resistanceConfig = config;
        onResistanceDone = onDone;
        currentSampleIndex = 0;
        accumulatedCurrent = 0.0f;

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
                    CalculateResistance();
            });
    }

    void MotorIdentificationImpl::CalculateResistance()
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

    void MotorIdentificationImpl::GetInductance(const InductanceConfig& config, const infra::Function<void(std::optional<foc::Henry>)>& onDone)
    {
        inductanceConfig = config;
        onInductanceDone = onDone;
        currentSampleIndex = 0;
        firstCurrent = 0.0f;
        secondCurrent = 0.0f;

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
                    CalculateInductance();
                }
                currentSampleIndex++;
            });
    }

    void MotorIdentificationImpl::CalculateInductance()
    {
        driver.Stop();

        float di = secondCurrent - firstCurrent;
        float dt = 1.0f / static_cast<float>(inductanceConfig.samplingFrequency.Value());
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

    void MotorIdentificationImpl::GetNumberOfPolePairs(const PolePairsConfig& config, const infra::Function<void(std::optional<std::size_t>)>& onDone)
    {
        polePairsConfig = config;
        onPolePairsDone = onDone;
        currentSampleIndex = 0;

        initialPosition = encoder.Read();

        ApplyNextElectricalAngle();
    }

    void MotorIdentificationImpl::ApplyNextElectricalAngle()
    {
        const std::size_t totalSteps = polePairsConfig.electricalRevolutions * stepsPerRevolution;

        if (currentSampleIndex < totalSteps)
        {
            auto anglePerStep = twoPi / static_cast<float>(stepsPerRevolution);
            auto electricalAngle = static_cast<float>(currentSampleIndex) * anglePerStep;
            auto voltage = static_cast<float>(polePairsConfig.testVoltagePercent.Value()) / 100.0f;

            driver.Stop();
            driver.ThreePhasePwmOutput(NormalizedDutyCycles(transforms.Inverse(foc::RotatingFrame{ voltage, 0.0f }, std::cos(electricalAngle), std::sin(electricalAngle))));
            driver.PhaseCurrentsReady(hal::Hertz{ 100 }, [this](auto)
                {
                    currentSampleIndex++;
                    ApplyNextElectricalAngle();
                });
        }
        else
            CalculatePolePairs();
    }

    void MotorIdentificationImpl::CalculatePolePairs()
    {
        driver.Stop();
        finalPosition = encoder.Read();

        auto mechanicalRotation = std::abs((finalPosition - initialPosition).Value());

        if (onPolePairsDone)
        {
            if (mechanicalRotation > polePairsConfig.minMechanicalRotation.Value())
            {
                auto electricalRevolutions = static_cast<float>(polePairsConfig.electricalRevolutions);
                auto mechanicalRevolutions = mechanicalRotation / twoPi;

                auto polePairs = static_cast<std::size_t>(std::round(electricalRevolutions / mechanicalRevolutions));
                onPolePairsDone(std::make_optional<std::size_t>(polePairs));
            }
            else
                onPolePairsDone(std::nullopt);
        }
    }
}
