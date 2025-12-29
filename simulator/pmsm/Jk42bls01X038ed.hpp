#pragma once

#include "simulator/pmsm/Model.hpp"

namespace simulator
{
    struct JK42BLS01_X038ED
    {
        static constexpr PmsmModel::Parameters parameters{
            .R = 0.073f,     // 0.073 Ω (from datasheet)
            .Ld = 0.0005f,   // ~0.5 mH (estimated)
            .Lq = 0.0005f,   // ~0.5 mH (estimated)
            .psi_f = 0.007f, // 0.007 Wb (from Kt = 0.042 Nm/A)
            .p = 4.0f,       // 4 pole pairs (8 poles)
            .J = 0.0000075f, // ~7.5 g·cm² (estimated)
            .B = 0.00002f,   // Viscous damping (estimated)
            .Vdc = 24.0f,    // 24V rated voltage
            .T_load = 0.01f  // Load torque in simulation
        };
    };
}
