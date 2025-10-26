#pragma once

#include "application/foc/implementations/TransformsClarkePark.hpp"
#include "numerical/math/CompilerOptimizations.hpp"

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
            really_assert(voltagePhase.alpha >= -invSqrt3 && voltagePhase.alpha <= invSqrt3);
            really_assert(voltagePhase.beta >= -invSqrt3 && voltagePhase.beta <= invSqrt3);

            auto pattern = CalculateSwitchingTimes(voltagePhase.alpha, voltagePhase.beta);

            return Output{
                ClampDutyCycle(pattern.ta),
                ClampDutyCycle(pattern.tb),
                ClampDutyCycle(pattern.tc)
            };
        }

    private:
        OPTIMIZE_FOR_SPEED
        float ClampDutyCycle(float duty)
        {
            if (duty < zero)
                return zero;
            if (duty > one)
                return one;

            return duty;
        }

        struct SwitchingPattern
        {
            float ta;
            float tb;
            float tc;
        };

        OPTIMIZE_FOR_SPEED
        SwitchingPattern AddCommonInjection(SwitchingPattern pattern)
        {
            static const float _half = float(0.5f);

            float t0 = one - pattern.ta - pattern.tb - pattern.tc;
            float t_com = t0 * _half;

            pattern.ta += t_com;
            pattern.tb += t_com;
            pattern.tc += t_com;

            return pattern;
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate60To120Degrees(float valpha, float vbeta)
        {
            float t1 = valpha + vbeta * invSqrt3;
            float t2 = -valpha + vbeta * invSqrt3;
            return AddCommonInjection({ t2, t1 + t2, zero });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate120To180Degrees(float valpha, float vbeta)
        {
            float t1 = -valpha + vbeta * invSqrt3;
            float t2 = -valpha - vbeta * invSqrt3;
            return AddCommonInjection({ zero, t1 + t2, t2 });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate180To240Degrees(float valpha, float vbeta)
        {
            float t1 = -valpha - vbeta * invSqrt3;
            float t2 = vbeta / sqrt3Div2;
            return AddCommonInjection({ zero, t2, t1 + t2 });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate240To300Degrees(float valpha, float vbeta)
        {
            float t1 = -valpha + vbeta * invSqrt3;
            float t2 = -vbeta / sqrt3Div2;
            return AddCommonInjection({ t2, zero, t1 + t2 });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate300To360Degrees(float valpha, float vbeta)
        {
            float t1 = valpha + vbeta * invSqrt3;
            float t2 = -valpha - vbeta * invSqrt3;
            return AddCommonInjection({ t1 + t2, zero, t2 });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate0To60Degrees(float valpha, float vbeta)
        {
            float t1 = valpha - vbeta * invSqrt3;
            float t2 = vbeta / sqrt3Div2;
            return AddCommonInjection({ t1 + t2, t2, zero });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern CalculateSwitchingTimes(float valpha, float vbeta)
        {
            float v_ref_60 = (valpha * half - vbeta * sqrt3Div2);
            float v_ref_120 = (-valpha * half - vbeta * sqrt3Div2);

            if (v_ref_60 >= zero)
                return Calculate60To120Degrees(valpha, vbeta);
            if (v_ref_120 >= zero)
                return Calculate120To180Degrees(valpha, vbeta);
            if (-valpha >= zero)
                return Calculate180To240Degrees(valpha, vbeta);
            if (-v_ref_60 >= zero)
                return Calculate240To300Degrees(valpha, vbeta);
            if (-v_ref_120 >= zero)
                return Calculate300To360Degrees(valpha, vbeta);

            return Calculate0To60Degrees(valpha, vbeta);
        }

        float zero{ float(0.0f) };
        float one{ float(0.9999f) };
        float half{ float(0.5f) };
        float invSqrt3{ float(0.577350269189625f) };
        float sqrt3Div2{ float(0.866025403784438f) };
    };
}
