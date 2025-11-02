
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
            float R = 0.5f;     // Phase resistance [Ohm]
            float Ld = 0.001f;  // d-axis inductance [H]
            float Lq = 0.001f;  // q-axis inductance [H] (SPM: Ld ≈ Lq)
            float psi_f = 0.1f; // Permanent magnet flux linkage [Wb]
            float p = 4.0f;     // Pole pairs

            // Mechanical parameters
            float J = 0.001;  // Rotor inertia [kg·m²]
            float B = 0.0001; // Viscous friction coefficient [N·m·s/rad]

            // Operating conditions
            float Vdc = 72.0f;   // DC bus voltage [V]
            float T_load = 0.1f; // Load torque [N·m]
        };

        PmsmModel(const Parameters& params, float timeStep);

        std::pair<std::tuple<foc::Ampere, foc::Ampere, foc::Ampere>, foc::Radians> Run(std::tuple<hal::Percent, hal::Percent, hal::Percent> dutyCycles);

    private:
        Parameters parameters;
        float dt;

        float ia = 0.0f;         // Phase a current [A]
        float ib = 0.0f;         // Phase b current [A]
        float ic = 0.0f;         // Phase c current [A]
        float theta = 0.0f;      // Rotor electrical angle [rad]
        float theta_mech = 0.0f; // Rotor mechanical angle [rad]
        float omega = 0.0f;      // Rotor electrical angular velocity [rad/s]
        float omega_mech = 0.0f; // Rotor mechanical angular velocity [rad/s]
    };
}
