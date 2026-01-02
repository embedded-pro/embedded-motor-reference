#pragma once

#include "infra/util/AutoResetFunction.hpp"
#include "source/foc/implementations/TransformsClarkePark.hpp"
#include "source/foc/interfaces/Driver.hpp"
#include "source/services/parameter_identification/MotorIdentification.hpp"

namespace services
{
    class MotorIdentificationImpl
        : public MotorIdentification
    {
    public:
        MotorIdentificationImpl(foc::MotorDriver& driver, foc::Encoder& encoder, foc::Volts vdc);

        void GetResistance(const ResistanceConfig& config, const infra::Function<void(std::optional<foc::Ohm>)>& onDone) override;
        void GetInductance(const InductanceConfig& config, const infra::Function<void(std::optional<foc::Henry>)>& onDone) override;
        void GetNumberOfPolePairs(const PolePairsConfig& config, const infra::Function<void(std::optional<std::size_t>)>& onDone) override;

    private:
        void CalculateResistance();
        void CalculateInductance();
        void CalculatePolePairs();
        void ApplyNextElectricalAngle();

        constexpr static uint8_t neutralDuty = 50;

        foc::MotorDriver& driver;
        foc::Encoder& encoder;
        foc::Volts vdc;
        [[no_unique_address]] foc::ClarkePark transforms;
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
    };
}
