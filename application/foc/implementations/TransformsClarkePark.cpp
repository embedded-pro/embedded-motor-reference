#include "application/foc/implementations/TransformsClarkePark.hpp"

namespace foc
{
    TwoPhase Clarke::Forward(const ThreePhase& input) const
    {
        const float bc_sum = input.b + input.c;
        return TwoPhase{ twoThirds * (input.a - oneHalf * bc_sum), invSqrt3 * (input.b - input.c) };
    }

    ThreePhase Clarke::Inverse(const TwoPhase& input) const
    {
        const float alpha_half = oneHalf * input.alpha;
        const float beta_sqrt3_half = sqrt3Div2 * input.beta;
        return ThreePhase{ input.alpha, -alpha_half + beta_sqrt3_half, -alpha_half - beta_sqrt3_half };
    }

    RotatingFrame Park::Forward(const TwoPhase& input, const float& cosTheta, const float& sinTheta) const
    {
        const float alpha_cos = input.alpha * cosTheta;
        const float beta_sin = input.beta * sinTheta;
        const float alpha_sin = input.alpha * sinTheta;
        const float beta_cos = input.beta * cosTheta;

        return RotatingFrame{ alpha_cos + beta_sin, -alpha_sin + beta_cos };
    }

    TwoPhase Park::Inverse(const RotatingFrame& input, const float& cosTheta, const float& sinTheta) const
    {
        const float d_cos = input.d * cosTheta;
        const float q_sin = input.q * sinTheta;
        const float d_sin = input.d * sinTheta;
        const float q_cos = input.q * cosTheta;

        return TwoPhase{ d_cos - q_sin, d_sin + q_cos };
    }

    RotatingFrame ClarkePark::Forward(const ThreePhase& input, const float& cosTheta, const float& sinTheta) const
    {
        return park.Forward(clarke.Forward(input), cosTheta, sinTheta);
    }

    ThreePhase ClarkePark::Inverse(const RotatingFrame& input, const float& cosTheta, const float& sinTheta) const
    {
        return clarke.Inverse(park.Inverse(input, cosTheta, sinTheta));
    }
}
