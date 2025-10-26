#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/TrigonometricFunctions.hpp"

namespace foc
{
    struct ThreePhase
    {
        float a;
        float b;
        float c;
    };

    struct TwoPhase
    {
        float alpha;
        float beta;
    };

    struct RotatingFrame
    {
        float d;
        float q;
    };

    class Clarke
    {
    public:
        OPTIMIZE_FOR_SPEED
        TwoPhase Forward(const ThreePhase& input)
        {
            const float bc_sum = input.b + input.c;

            return TwoPhase{ twoThirds * (input.a - oneHalf * bc_sum), invSqrt3 * (input.b - input.c) };
        }

        OPTIMIZE_FOR_SPEED
        ThreePhase Inverse(const TwoPhase& input)
        {
            const float alpha_half = oneHalf * input.alpha;
            const float beta_sqrt3_half = sqrt3Div2 * input.beta;
            return ThreePhase{ input.alpha, -alpha_half + beta_sqrt3_half, -alpha_half - beta_sqrt3_half };
        }

    private:
        float oneHalf = float(0.5f);
        float twoThirds = float(0.666666667f);
        float invSqrt3 = float(0.577350269f);
        float sqrt3Div2 = float(0.8660254037f);
    };

    class Park
    {
    public:
        explicit Park(const math::TrigonometricFunctions<float>& trigFunctions)
            : trigFunctions(trigFunctions)
        {}

        OPTIMIZE_FOR_SPEED
        RotatingFrame Forward(const TwoPhase& input, float scaledTheta)
        {
            const float cosTheta = trigFunctions.Cosine(scaledTheta);
            const float sinTheta = trigFunctions.Sine(scaledTheta);

            const float alpha_cos = input.alpha * cosTheta;
            const float beta_sin = input.beta * sinTheta;
            const float alpha_sin = input.alpha * sinTheta;
            const float beta_cos = input.beta * cosTheta;

            return RotatingFrame{ alpha_cos + beta_sin, -alpha_sin + beta_cos };
        }

        OPTIMIZE_FOR_SPEED
        TwoPhase Inverse(const RotatingFrame& input, float scaledTheta)
        {
            const float cosTheta = trigFunctions.Cosine(scaledTheta);
            const float sinTheta = trigFunctions.Sine(scaledTheta);

            const float d_cos = input.d * cosTheta;
            const float q_sin = input.q * sinTheta;
            const float d_sin = input.d * sinTheta;
            const float q_cos = input.q * cosTheta;

            return TwoPhase{ d_cos - q_sin, d_sin + q_cos };
        }

    private:
        const math::TrigonometricFunctions<float>& trigFunctions;
    };

    class ClarkePark
    {
    public:
        explicit ClarkePark(const math::TrigonometricFunctions<float>& trigFunctions)
            : park(trigFunctions)
        {}

        OPTIMIZE_FOR_SPEED
        RotatingFrame Forward(const ThreePhase& input, float theta)
        {
            return park.Forward(clarke.Forward(input), theta);
        }

        OPTIMIZE_FOR_SPEED
        ThreePhase Inverse(const RotatingFrame& input, float theta)
        {
            return clarke.Inverse(park.Inverse(input, theta));
        }

    private:
        Clarke clarke;
        Park park;
    };
}
