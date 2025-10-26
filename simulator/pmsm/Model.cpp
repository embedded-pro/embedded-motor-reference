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
        constexpr float one_half = 0.5f;
        constexpr float sqrt3_over_2 = 0.866025f;
        constexpr float one_third = 0.3333333f;
    }

    PmsmModel::PmsmModel(const Parameters& params, float timeStep)
        : parameters(params)
        , dt(timeStep)
    {
    }

    std::pair<std::tuple<foc::Ampere, foc::Ampere, foc::Ampere>, foc::Radians> PmsmModel::Run(std::tuple<hal::Percent, hal::Percent, hal::Percent> dutyCycles)
    {
        // Convert duty cycles (0-100%) to phase voltages
        auto duty_a = std::get<0>(dutyCycles).Value() / 100.0f;
        auto duty_b = std::get<1>(dutyCycles).Value() / 100.0f;
        auto duty_c = std::get<2>(dutyCycles).Value() / 100.0f;

        // Phase voltages referenced to DC bus midpoint
        auto va = (duty_a - one_half) * parameters.Vdc;
        auto vb = (duty_b - one_half) * parameters.Vdc;
        auto vc = (duty_c - one_half) * parameters.Vdc;

        // Calculate electrical angle from mechanical angle
        float theta_e = theta_m * parameters.p;

        // Wrap electrical angle to [0, 2π]
        while (theta_e >= two_pi)
            theta_e -= two_pi;
        while (theta_e < 0.0f)
            theta_e += two_pi;

        float cos_theta = std::cos(theta_e);
        float sin_theta = std::sin(theta_e);

        // Clarke Transform: ABC -> αβ
        float v_alpha = two_thirds * (va - 0.5f * vb - 0.5f * vc);
        float v_beta = two_thirds * (sqrt3_over_2 * vb - sqrt3_over_2 * vc);

        // Park Transform: αβ -> dq
        float vd = v_alpha * cos_theta + v_beta * sin_theta;
        float vq = -v_alpha * sin_theta + v_beta * cos_theta;

        // Electrical model in dq frame (voltage equations)
        // vd = R*id + Ld*did/dt - omega_e*Lq*iq
        // vq = R*iq + Lq*diq/dt + omega_e*(Ld*id + psi_f)

        float omega_e = omega * parameters.p; // Electrical angular velocity

        // Calculate derivatives using backward Euler integration
        float did_dt = (vd - parameters.R * id + omega_e * parameters.Lq * iq) / parameters.Ld;
        float diq_dt = (vq - parameters.R * iq - omega_e * (parameters.Ld * id + parameters.psi_f)) / parameters.Lq;

        // Integrate currents
        id += did_dt * dt;
        iq += diq_dt * dt;

        // Calculate electromagnetic torque
        // T_em = 1.5 * p * (psi_f * iq + (Ld - Lq) * id * iq)
        float T_em = 1.5f * parameters.p * (parameters.psi_f * iq + (parameters.Ld - parameters.Lq) * id * iq);

        // Mechanical model (equation of motion)
        // J * domega/dt = T_em - B * omega - T_load
        float domega_dt = (T_em - parameters.B * omega - parameters.T_load) / parameters.J;

        // Integrate mechanical speed and position
        omega += domega_dt * dt;
        theta_m += omega * dt;

        // Wrap mechanical angle to [0, 2π]
        while (theta_m >= two_pi)
            theta_m -= two_pi;
        while (theta_m < 0.0f)
            theta_m += two_pi;

        // Inverse Park Transform: dq -> αβ
        float i_alpha = id * cos_theta - iq * sin_theta;
        float i_beta = id * sin_theta + iq * cos_theta;

        // Inverse Clarke Transform: αβ -> ABC
        float ia_val = i_alpha;
        float ib_val = -0.5f * i_alpha + sqrt3_over_2 * i_beta;
        float ic_val = -0.5f * i_alpha - sqrt3_over_2 * i_beta;

        // Convert back to electrical angle for output
        float theta_e_out = theta_m * parameters.p;
        while (theta_e_out >= two_pi)
            theta_e_out -= two_pi;
        while (theta_e_out < 0.0f)
            theta_e_out += two_pi;

        return { std::make_tuple(
                     foc::Ampere{ ia_val },
                     foc::Ampere{ ib_val },
                     foc::Ampere{ ic_val }),
            foc::Radians{ theta_e_out } };
    }
}
