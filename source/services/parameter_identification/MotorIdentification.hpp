#pragma once

#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "infra/util/Function.hpp"
#include "source/foc/interfaces/Units.hpp"
#include <optional>

namespace services
{
    class MotorIdentification
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

        virtual void GetResistance(const ResistanceConfig& config, const infra::Function<void(std::optional<foc::Ohm>)>& onDone) = 0;
        virtual void GetInductance(const InductanceConfig& config, const infra::Function<void(std::optional<foc::Henry>)>& onDone) = 0;
        virtual void GetNumberOfPolePairs(const PolePairsConfig& config, const infra::Function<void(std::optional<std::size_t>)>& onDone) = 0;
    };
}
