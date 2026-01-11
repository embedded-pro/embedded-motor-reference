#include "source/services/parameter_identification/MotorIdentificationImpl.hpp"
#include "source/foc/interfaces/Units.hpp"
#include <cmath>
#include <numbers>
#include <numeric>

namespace
{
    constexpr float twoPi = 2.0f * std::numbers::pi_v<float>;
    constexpr std::size_t stepsPerRevolution = 12;
    constexpr auto anglePerStep = twoPi / static_cast<float>(stepsPerRevolution);
    constexpr float minRotationThreshold = std::numbers::pi_v<float> / 2.0f;
    constexpr float timeConstantThreshold = 0.632f;
    hal::Hertz samplingFrequency{ 10000 };
    auto samplingPeriod = 1.0f / static_cast<float>(samplingFrequency.Value());

    float SimulateRLModelCurrent(float voltage, float resistance, float inductance, float time)
    {
        if (resistance <= 0.0f)
            return 0.0f;

        float tau = inductance / resistance;
        float steadyStateCurrent = voltage / resistance;

        return steadyStateCurrent * (1.0f - std::exp(-time / tau));
    }

    foc::PhasePwmDutyCycles NormalizedDutyCycles(foc::ThreePhase voltages)
    {
        auto offset = 50.0f;
        auto dutyA = static_cast<uint8_t>(std::clamp(offset + voltages.a * 50.0f, 0.0f, 100.0f));
        auto dutyB = static_cast<uint8_t>(std::clamp(offset + voltages.b * 50.0f, 0.0f, 100.0f));
        auto dutyC = static_cast<uint8_t>(std::clamp(offset + voltages.c * 50.0f, 0.0f, 100.0f));
        return foc::PhasePwmDutyCycles{ hal::Percent{ dutyA }, hal::Percent{ dutyB }, hal::Percent{ dutyC } };
    }

    float AverageAndRemoveFront(infra::BoundedDeque<float>& deque)
    {
        float sum = 0.0f;

        for (auto& samples : deque)
            sum += samples;

        float average = sum / static_cast<float>(deque.size());

        deque.pop_front();

        return average;
    }

    float GetSteadyStateCurrent(const infra::BoundedVector<float>& samples)
    {
        auto lastQuarter = static_cast<std::size_t>(static_cast<float>(samples.size()) * 0.9f);

        return std::accumulate(samples.begin() + lastQuarter, samples.end(), 0.0f) / static_cast<float>(samples.size() - lastQuarter);
    }

    std::optional<float> GetTauFromCurrentSamples(const infra::BoundedVector<float>& samples, float steadyStateCurrent, std::size_t averageFilter)
    {
        auto targetCurrent = timeConstantThreshold * steadyStateCurrent;

        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            if (samples[i] >= targetCurrent)
            {
                if (i >= averageFilter)
                    return static_cast<float>(i - averageFilter);
                else
                    return static_cast<float>(i);
            }
        }

        return std::nullopt;
    }

    std::optional<foc::Ohm> CalculateResistance(float voltage, float current)
    {
        return foc::Ohm{ voltage / current };
    }

    std::optional<foc::MilliHenry> CalculateInductance(foc::Ohm resistance, float tau)
    {
        return foc::MilliHenry{ resistance.Value() * tau * samplingPeriod * 1000.0f };
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

    void MotorIdentificationImpl::EstimateResistanceAndInductance(const ResistanceAndInductanceConfig& config, const infra::Function<void(std::optional<foc::Ohm>, std::optional<foc::MilliHenry>)>& onDone)
    {
        resistanceAndInductanceConfig = config;
        onResistanceAndInductanceDone = onDone;
        currentSamples.clear();
        filteredCurrentSample.clear();

        driver.PhaseCurrentsReady(samplingFrequency, [](auto) {});
        driver.ThreePhasePwmOutput(foc::PhasePwmDutyCycles{
            hal::Percent{ neutralDuty },
            hal::Percent{ neutralDuty },
            hal::Percent{ neutralDuty } });

        settleTimer.Start(resistanceAndInductanceConfig.settleTime, [this]()
            {
                driver.PhaseCurrentsReady(samplingFrequency, [this](auto currentPhases)
                    {
                        currentSamples.push_back(currentPhases.a.Value());

                        if (currentSamples.full())
                            filteredCurrentSample.push_back(AverageAndRemoveFront(currentSamples));

                        if (filteredCurrentSample.full())
                            AnalyzeInductanceMeasures();
                    });

                driver.ThreePhasePwmOutput(foc::PhasePwmDutyCycles{
                    hal::Percent{ resistanceAndInductanceConfig.testVoltagePercent.Value() },
                    hal::Percent{ neutralDuty },
                    hal::Percent{ neutralDuty } });
            });
    }

    void MotorIdentificationImpl::AnalyzeInductanceMeasures()
    {
        driver.Stop();

        auto steadyStateCurrent = GetSteadyStateCurrent(filteredCurrentSample);

        if (steadyStateCurrent <= 0.0f)
            onResistanceAndInductanceDone(std::nullopt, std::nullopt);
        else
        {
            auto tau = GetTauFromCurrentSamples(filteredCurrentSample, steadyStateCurrent, averageFilter);
            auto resistance = CalculateResistance(static_cast<float>(resistanceAndInductanceConfig.testVoltagePercent.Value() * vdc.Value() / 100.0f), steadyStateCurrent);

            if (resistance.has_value() && tau.has_value())
                onResistanceAndInductanceDone(resistance, CalculateInductance(resistance.value(), tau.value_or(0.0f)));
            else
                onResistanceAndInductanceDone(std::nullopt, std::nullopt);

            filteredCurrentSample.clear();
        }
    }

    void MotorIdentificationImpl::EstimateNumberOfPolePairs(const PolePairsConfig& config, const infra::Function<void(std::optional<std::size_t>)>& onDone)
    {
        polePairsConfig = config;
        onPolePairsDone = onDone;
        currentSampleIndex = 0;
        accumulatedRotation = 0.0f;

        initialPosition = encoder.Read();
        previousPosition = initialPosition;

        ApplyNextElectricalAngle();
    }

    void MotorIdentificationImpl::ApplyNextElectricalAngle()
    {
        const std::size_t totalSteps = polePairsConfig.electricalRevolutions * stepsPerRevolution;

        if (currentSampleIndex < totalSteps)
            RunPolePairLogic();
        else
            CalculatePolePairs();
    }

    void MotorIdentificationImpl::RunPolePairLogic()
    {
        auto electricalAngle = static_cast<float>(currentSampleIndex) * anglePerStep;

        driver.PhaseCurrentsReady(samplingFrequency, [this](auto currents)
            {
                auto currentPosition = encoder.Read();
                auto delta = currentPosition.Value() - previousPosition.Value();

                delta = delta - twoPi * std::floor((delta + std::numbers::pi_v<float>) / twoPi);

                accumulatedRotation += delta;
                previousPosition = currentPosition;

                currentSampleIndex++;
                ApplyNextElectricalAngle();
            });
        auto voltage = static_cast<float>(polePairsConfig.testVoltagePercent.Value()) * vdc.Value() / 100.0f;
        driver.ThreePhasePwmOutput(NormalizedDutyCycles(transforms.Inverse(foc::RotatingFrame{ voltage, 0.0f }, std::cos(electricalAngle), std::sin(electricalAngle))));
    }

    void MotorIdentificationImpl::CalculatePolePairs()
    {
        driver.Stop();

        if (onPolePairsDone)
        {
            auto mechanicalRotation = std::abs(accumulatedRotation);

            if (mechanicalRotation > minRotationThreshold)
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
