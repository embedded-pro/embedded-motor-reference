#include "source/foc/implementations/SpaceVectorModulation.hpp"
#include <algorithm>

namespace foc
{
    SpaceVectorModulation::Output SpaceVectorModulation::Generate(const TwoPhase& voltagePhase) const
    {
        auto vA = voltagePhase.alpha;
        auto vB = (-voltagePhase.alpha * half + voltagePhase.beta * sqrt3Div2);
        auto vC = (-voltagePhase.alpha * half - voltagePhase.beta * sqrt3Div2);

        auto vMax = std::max({ vA, vB, vC });
        auto vMin = std::min({ vA, vB, vC });
        auto vCommon = (vMax + vMin) * -half;

        return Output{ Clip(vA + vCommon), Clip(vB + vCommon), Clip(vC + vCommon) };
    }

    float SpaceVectorModulation::Clip(float dutyCycle) const
    {
        return std::clamp(dutyCycle * invSqrt3 + half, zero, one);
    }
}
