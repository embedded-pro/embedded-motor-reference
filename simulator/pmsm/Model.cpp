#include "simulator/pmsm/Model.hpp"
#include "application/foc/interfaces/Driver.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include <cmath>

namespace simulator
{
    namespace
    {
        constexpr float two_pi = 6.28318530718f;
        constexpr float two_thirds = 0.6666667f;
        constexpr float half = 0.5f;
        constexpr float sqrt3_over_2 = 0.866025f;
        constexpr float sqrt3_over_3 = 0.577350269189625f;
    }

    PmsmModel::PmsmModel(const Parameters& params, float timeStep)
        : parameters(params)
        , dt(timeStep)
    {
    }

    std::pair<foc::PhaseCurrents, foc::Radians> PmsmModel::Run(foc::PhasePwmDutyCycles dutyCycles)
    {
        auto duty_a = dutyCycles.a.Value() / 100.0f;
        auto duty_b = dutyCycles.b.Value() / 100.0f;
        auto duty_c = dutyCycles.c.Value() / 100.0f;

        auto va = (duty_a - half) * parameters.Vdc;
        auto vb = (duty_b - half) * parameters.Vdc;
        auto vc = (duty_c - half) * parameters.Vdc;

        auto v_alpha = two_thirds * (va - half * (vb + vc));
        auto v_beta = sqrt3_over_3 * (vb - vc);

        auto cos_theta = std::cos(theta);
        auto sin_theta = std::sin(theta);

        auto vd = v_alpha * cos_theta + v_beta * sin_theta;
        auto vq = -v_alpha * sin_theta + v_beta * cos_theta;

        auto i_alpha = two_thirds * (ia - half * (ib + ic));
        auto i_beta = sqrt3_over_3 * (ib - ic);

        auto id = i_alpha * cos_theta + i_beta * sin_theta;
        auto iq = -i_alpha * sin_theta + i_beta * cos_theta;

        auto dId_dt = (vd - parameters.R * id + omega * parameters.Lq * iq) / parameters.Ld;
        auto dIq_dt = (vq - parameters.R * iq - omega * parameters.Ld * id - omega * parameters.psi_f) / parameters.Lq;

        id += dId_dt * dt;
        iq += dIq_dt * dt;

        i_alpha = id * cos_theta - iq * sin_theta;
        i_beta = id * sin_theta + iq * cos_theta;

        ia = i_alpha;
        ib = -half * i_alpha + sqrt3_over_2 * i_beta;
        ic = -half * i_alpha - sqrt3_over_2 * i_beta;

        auto torqueElec = 1.5f * parameters.p * (parameters.psi_f * iq + (parameters.Ld - parameters.Lq) * id * iq);
        auto d_omega_mech_dt = (torqueElec - parameters.B * omega_mech - parameters.T_load) / parameters.J;

        omega_mech += d_omega_mech_dt * dt;
        omega = parameters.p * omega_mech;

        theta_mech += omega_mech * dt;
        theta += omega * dt;

        theta_mech = std::fmod(theta_mech, two_pi);
        theta = std::fmod(theta, two_pi);

        return { { foc::Ampere{ ia }, foc::Ampere{ ib }, foc::Ampere{ ic } }, foc::Radians{ theta_mech } };
    }
}
