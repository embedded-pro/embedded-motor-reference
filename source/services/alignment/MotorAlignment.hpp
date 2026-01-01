#pragma once

#include "infra/util/AutoResetFunction.hpp"
#include "infra/util/Function.hpp"
#include "source/foc/implementations/TransformsClarkePark.hpp"
#include "source/foc/interfaces/Driver.hpp"
#include "source/foc/interfaces/Units.hpp"
#include <optional>

namespace services
{
    class MotorAlignment
    {
    public:
        struct AlignmentConfig
        {
            hal::Percent testVoltagePercent{ 20 };
            hal::Hertz samplingFrequency{ 1000 };
            std::size_t maxSamples{ 500 };
            foc::Radians settledThreshold{ 0.001f };
            std::size_t settledCount{ 10 };
        };

        MotorAlignment(foc::MotorDriver& driver, foc::Encoder& encoder, foc::Volts vdc);

        void ForceAlignment(std::size_t polePairs, const AlignmentConfig& config, const infra::Function<void(std::optional<foc::Radians>)>& onDone);

    private:
        void ApplyAlignmentVoltage();
        void CalculateAlignmentOffset();
        void ProcessPosition();
        void FailToConverge();

        constexpr static uint8_t neutralDuty = 50;
        constexpr static float alignmentAngle = 0.0f;

        foc::MotorDriver& driver;
        foc::Encoder& encoder;
        foc::Volts vdc;
        [[no_unique_address]] foc::ClarkePark transforms;
        AlignmentConfig alignmentConfig;
        std::size_t polePairs = 1;
        std::size_t currentSampleIndex = 0;
        std::size_t consecutiveSettledSamples = 0;
        foc::Radians previousPosition{ 0.0f };
        foc::Radians alignedPosition{ 0.0f };
        infra::AutoResetFunction<void(std::optional<foc::Radians>)> onAlignmentDone;
    };
}
