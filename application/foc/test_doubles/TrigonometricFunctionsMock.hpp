#pragma once

#include "numerical/math/TrigonometricFunctions.hpp"
#include <gmock/gmock.h>

namespace application
{
    class TrigonometricFunctionsMock
        : public math::TrigonometricFunctions<float>
    {
    public:
        MOCK_METHOD(float, Cosine, (const float& angle), (const, override));
        MOCK_METHOD(float, Sine, (const float& angle), (const, override));
        MOCK_METHOD(float, Arctangent, (const float& value), (const, override));
        MOCK_METHOD(float, Phase, (const float& real, const float& imag), (const, override));
    };
}
