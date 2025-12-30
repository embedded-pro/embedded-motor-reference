#pragma once

#include "infra/util/AutoResetFunction.hpp"
#include "infra/util/Function.hpp"
#include "source/foc/implementations/TransformsClarkePark.hpp"
#include "source/foc/interfaces/Driver.hpp"
#include "source/foc/interfaces/Units.hpp"
#include <optional>

namespace services
{
    class MotorIdentificationWithAlignment
    {
    public:
        struct ResistanceConfig
        {
            hal::Percent testVoltagePercent{ 5 };
            std::size_t sampleCount{ 16 };
            foc::Ampere minCurrent{ 0.1f };
        };

        struct InductanceConfig
        {
            hal::Percent testVoltagePercent{ 10 };
            foc::Ohm resistance{ 1.0f };
            hal::Hertz samplingFrequency{ 10000 };
            foc::Ampere minCurrentChange{ 0.5f };
        };

        struct PolePairsConfig
        {
            hal::Percent testVoltagePercent{ 20 };
            std::size_t electricalRevolutions{ 1 };
            foc::Radians minMechanicalRotation{ 0.1f };
        };

        MotorIdentificationWithAlignment(foc::MotorDriver& driver, foc::Encoder& encoder, foc::Volts vdc);

        void GetResistance(const ResistanceConfig& config, const infra::Function<void(std::optional<foc::Ohm>)>& onDone);
        void GetInductance(const InductanceConfig& config, const infra::Function<void(std::optional<foc::Henry>)>& onDone);
        void GetNumberOfPolePairs(const PolePairsConfig& config, const infra::Function<void(std::optional<std::size_t>)>& onDone);

        void AlignMotor(std::size_t polePairs, const infra::Function<void(std::optional<foc::Radians>)>& onDone);

    private:
        void CalculateResistance(foc::PhaseCurrents currentPhases);
        void CalculateInductance(foc::PhaseCurrents currentPhases);
        void CalculatePolePairs();
        void ApplyNextElectricalAngle();

        constexpr static uint8_t neutralDuty = 50;

        foc::MotorDriver& driver;
        foc::Encoder& encoder;
        foc::Volts vdc;
        foc::ClarkePark transforms;
        ResistanceConfig resistanceConfig;
        InductanceConfig inductanceConfig;
        PolePairsConfig polePairsConfig;
        std::size_t currentSampleIndex = 0;
        float accumulatedCurrent = 0.0f;
        float firstCurrent = 0.0f;
        float secondCurrent = 0.0f;
        foc::Radians initialPosition{ 0.0f };
        foc::Radians finalPosition{ 0.0f };
        infra::AutoResetFunction<void(std::optional<foc::Ohm>)> onResistanceDone;
        infra::AutoResetFunction<void(std::optional<foc::Henry>)> onInductanceDone;
        infra::AutoResetFunction<void(std::optional<std::size_t>)> onPolePairsDone;
        infra::AutoResetFunction<void(std::optional<foc::Radians>)> onAlignmentDone;
    };
}
