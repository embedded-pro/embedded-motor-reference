#pragma once

#include "numerical/math/TrigonometricFunctions.hpp"

namespace foc
{
    class TrigonometricFunctions
        : public math::TrigonometricFunctions<float>
    {
    public:
        float Cosine(const float& angle) const override;
        float Sine(const float& angle) const override;
        float Arctangent(const float& value) const override;
        float Phase(const float& real, const float& imag) const override;
    };
}
