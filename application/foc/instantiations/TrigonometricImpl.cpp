#include "application/foc/instantiations/TrigonometricImpl.hpp"
#include <cmath>

namespace application
{
    float TrigonometricFunctions::Cosine(const float& angle) const
    {
        return std::cos(angle);
    }

    float TrigonometricFunctions::Sine(const float& angle) const
    {
        return std::sin(angle);
    }

    float TrigonometricFunctions::Arctangent(const float& value) const
    {
        return std::atan(value);
    }

    float TrigonometricFunctions::Phase(const float& real, const float& imag) const
    {
        return std::atan2(imag, real);
    }
}
