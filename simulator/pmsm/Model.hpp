
#pragma once

#include "source/foc/interfaces/Driver.hpp"
#include <utility>

namespace simulator
{
    class PmsmModel
    {
    public:
        struct Parameters
        {
            // Motor electrical parameters (based on typical small PMSM, similar to Maxon EC 45)
            float R;     // Phase resistance [Ohm]
            float Ld;    // d-axis inductance [H]
            float Lq;    // q-axis inductance [H] (SPM: Ld ≈ Lq)
            float psi_f; // Permanent magnet flux linkage [Wb]
            float p;     // Pole pairs

            // Mechanical parameters
            float J; // Rotor inertia [kg·m²]
            float B; // Viscous friction coefficient [N·m·s/rad]

            // Operating conditions
            float Vdc;    // DC bus voltage [V]
            float T_load; // Load torque [N·m]
        };

        PmsmModel(const Parameters& params, float timeStep);

        std::pair<foc::PhaseCurrents, foc::Radians> Run(foc::PhasePwmDutyCycles dutyCycles);

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
