
#pragma once

#include "application/foc/interfaces/Driver.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include <tuple>
#include <utility>

namespace simulator
{
    class PmsmModel
    {
    public:
        struct Parameters
        {
            // Motor electrical parameters (based on typical small PMSM, similar to Maxon EC 45)
            float R = 0.86f;       // Phase resistance [Ohm]
            float Ld = 0.365e-3f;  // d-axis inductance [H]
            float Lq = 0.365e-3f;  // q-axis inductance [H] (SPM: Ld ≈ Lq)
            float psi_f = 0.0253f; // Permanent magnet flux linkage [Wb]
            float p = 4.0f;        // Pole pairs

            // Mechanical parameters
            float J = 1.02e-5f; // Rotor inertia [kg·m²]
            float B = 1.0e-5f;  // Viscous friction coefficient [N·m·s/rad]

            // Operating conditions
            float Vdc = 48.0f;   // DC bus voltage [V]
            float T_load = 0.0f; // Load torque [N·m]
        };

        PmsmModel(const Parameters& params, float timeStep);

        std::pair<std::tuple<foc::Ampere, foc::Ampere, foc::Ampere>, foc::Radians> Run(std::tuple<hal::Percent, hal::Percent, hal::Percent> dutyCycles);

    private:
        Parameters parameters;
        float dt;

        // State variables
        float id = 0.0f;      // d-axis current [A]
        float iq = 0.0f;      // q-axis current [A]
        float omega = 0.0f;   // Mechanical angular velocity [rad/s]
        float theta_m = 0.1f; // Mechanical angle [rad] - small initial value to avoid singularity
        float int_id = 0.0f;  // Reserved for future use
        float int_iq = 0.0f;  // Reserved for future use
    };
}
