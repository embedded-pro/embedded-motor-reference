#pragma once

#include "application/foc/implementations/TransformsClarkePark.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <algorithm>

namespace foc
{
    class SpaceVectorModulation
    {
    public:
        struct Output
        {
            float a;
            float b;
            float c;
        };

        OPTIMIZE_FOR_SPEED
        Output Generate(const TwoPhase& voltagePhase)
        {
            auto vA = voltagePhase.alpha;
            auto vB = (-voltagePhase.alpha * half + voltagePhase.beta * sqrt3Div2);
            auto vC = (-voltagePhase.alpha * half - voltagePhase.beta * sqrt3Div2);

            auto vMax = std::max({ vA, vB, vC });
            auto vMin = std::min({ vA, vB, vC });
            auto vCommon = (vMax + vMin) * -half;

            return Output{ Clip(vA + vCommon), Clip(vB + vCommon), Clip(vC + vCommon) };
        }

    private:
        OPTIMIZE_FOR_SPEED
        float Clip(float dutyCycle)
        {
            return std::clamp(dutyCycle * invSqrt3 + half, zero, one);
        }

        static constexpr float zero{ float(0.0f) };
        static constexpr float one{ float(1.0f) };
        static constexpr float half{ float(0.5f) };
        static constexpr float invSqrt3{ float(0.577350269189625f) };
        static constexpr float sqrt3Div2{ float(0.866025403784438f) };
    };
}
