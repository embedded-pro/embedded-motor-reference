import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


# -------------------------------------------------------
# PMSM Model Class (matches C++ Model.cpp structure)
# -------------------------------------------------------
class PmsmModel:
    """
    PMSM electrical and mechanical model.
    Matches the C++ implementation in simulator/pmsm/Model.cpp
    """
    
    class Parameters:
        """Motor parameters (based on Maxon EC 45 flat motor)"""
        def __init__(self):
            self.R = 0.86          # Ohm - Phase resistance
            self.Ld = 0.365e-3     # H - d-axis inductance
            self.Lq = 0.365e-3     # H - q-axis inductance (SPM: Ld ≈ Lq)
            self.psi_f = 0.0253    # Wb - Permanent magnet flux linkage
            self.p = 4.0           # Pole pairs
            self.J = 1.02e-5       # kg·m² - Rotor inertia
            self.B = 1.0e-5        # N·m·s/rad - Viscous friction coefficient
            self.Vdc = 48.0        # DC bus voltage [V]
            self.T_load = 0.0      # Load torque [N·m]
    
    def __init__(self, params, time_step):
        self.parameters = params
        self.dt = time_step
        
        # State variables (matching C++ Model.hpp)
        self.id = 0.0
        self.iq = 0.0
        self.omega = 0.0
        self.theta_m = 0.1  # Small initial value to avoid singularity
        
    def run(self, duty_cycles):
        """
        Run one simulation step.
        Matches C++ signature: Run(tuple<Percent, Percent, Percent> dutyCycles)
        Returns: tuple of (currents_abc, electrical_angle)
        """
        duty_a, duty_b, duty_c = duty_cycles
        
        # Convert duty cycles (0-100%) to phase voltages
        va = (duty_a / 100.0 - 0.5) * self.parameters.Vdc
        vb = (duty_b / 100.0 - 0.5) * self.parameters.Vdc
        vc = (duty_c / 100.0 - 0.5) * self.parameters.Vdc
        
        # Calculate electrical angle from mechanical angle
        theta_e = self.theta_m * self.parameters.p
        
        # Wrap electrical angle to [0, 2π]
        theta_e = theta_e % (2.0 * np.pi)
        
        cos_theta = np.cos(theta_e)
        sin_theta = np.sin(theta_e)
        
        # Clarke Transform: ABC -> αβ
        v_alpha = (2.0/3.0) * (va - 0.5 * vb - 0.5 * vc)
        v_beta = (2.0/3.0) * (np.sqrt(3)/2 * vb - np.sqrt(3)/2 * vc)
        
        # Park Transform: αβ -> dq
        vd = v_alpha * cos_theta + v_beta * sin_theta
        vq = -v_alpha * sin_theta + v_beta * cos_theta
        
        # Electrical model in dq frame (voltage equations)
        # vd = R*id + Ld*did/dt - omega_e*Lq*iq
        # vq = R*iq + Lq*diq/dt + omega_e*(Ld*id + psi_f)
        
        omega_e = self.omega * self.parameters.p  # Electrical angular velocity
        
        # Calculate derivatives using backward Euler integration
        did_dt = (vd - self.parameters.R * self.id + omega_e * self.parameters.Lq * self.iq) / self.parameters.Ld
        diq_dt = (vq - self.parameters.R * self.iq - omega_e * (self.parameters.Ld * self.id + self.parameters.psi_f)) / self.parameters.Lq
        
        # Integrate currents
        self.id += did_dt * self.dt
        self.iq += diq_dt * self.dt
        
        # Calculate electromagnetic torque
        # T_em = 1.5 * p * (psi_f * iq + (Ld - Lq) * id * iq)
        T_em = 1.5 * self.parameters.p * (self.parameters.psi_f * self.iq + 
                                          (self.parameters.Ld - self.parameters.Lq) * self.id * self.iq)
        
        # Mechanical model (equation of motion)
        # J * domega/dt = T_em - B * omega - T_load
        domega_dt = (T_em - self.parameters.B * self.omega - self.parameters.T_load) / self.parameters.J
        
        # Integrate mechanical speed and position
        self.omega += domega_dt * self.dt
        self.theta_m += self.omega * self.dt
        
        # Wrap mechanical angle to [0, 2π]
        self.theta_m = self.theta_m % (2.0 * np.pi)
        
        # Inverse Park Transform: dq -> αβ
        i_alpha = self.id * cos_theta - self.iq * sin_theta
        i_beta = self.id * sin_theta + self.iq * cos_theta
        
        # Inverse Clarke Transform: αβ -> ABC
        ia = i_alpha
        ib = -0.5 * i_alpha + (np.sqrt(3)/2) * i_beta
        ic = -0.5 * i_alpha - (np.sqrt(3)/2) * i_beta
        
        # Convert back to electrical angle for output
        theta_e_out = (self.theta_m * self.parameters.p) % (2.0 * np.pi)
        
        return (ia, ib, ic), theta_e_out


# -------------------------------------------------------
# PID Controller Class (simplified velocity-form)
# -------------------------------------------------------
class PidController:
    """
    PI controller for current regulation.
    Simplified version matching the incremental PID concept.
    """
    
    def __init__(self, kp, ki, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.setpoint = 0.0
        
    def set_point(self, setpoint):
        self.setpoint = setpoint
        
    def process(self, measured_value, dt):
        """Process one step of PI control"""
        error = self.setpoint - measured_value
        
        # PI calculation
        proportional = self.kp * error
        self.integral += self.ki * error * dt
        
        # Calculate output
        output = proportional + self.integral
        
        # Clamp output (anti-windup)
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
            
        return output
    
    def reset(self):
        self.integral = 0.0


# -------------------------------------------------------
# Space Vector Modulation Class
# -------------------------------------------------------
class SpaceVectorModulation:
    """
    SVPWM implementation.
    Converts normalized αβ voltages to duty cycles.
    """
    
    def __init__(self):
        self.sqrt3 = np.sqrt(3)
        self.inv_sqrt3 = 1.0 / self.sqrt3
        
    def generate(self, v_alpha, v_beta):
        """
        Generate duty cycles from normalized voltage inputs.
        Input: Vα, Vβ normalized to DC bus voltage (range: [-0.577, 0.577])
        Output: Duty cycles in range [0.0, 1.0] representing 0% to 100%
        """
        # Scale inputs by √3
        valpha_scaled = v_alpha * self.sqrt3
        vbeta_scaled = v_beta * self.sqrt3
        
        # Determine sector and calculate switching times
        ta, tb, tc = self._calculate_switching_times(valpha_scaled, vbeta_scaled)
        
        # Clamp duty cycles to [0, 1]
        ta = np.clip(ta, 0.0, 1.0)
        tb = np.clip(tb, 0.0, 1.0)
        tc = np.clip(tc, 0.0, 1.0)
        
        return ta, tb, tc
    
    def _calculate_switching_times(self, valpha, vbeta):
        """Calculate switching times based on sector"""
        # Sector determination
        v_ref_60 = (valpha * 0.5 - vbeta * (self.sqrt3/2))
        v_ref_120 = (-valpha * 0.5 - vbeta * (self.sqrt3/2))
        
        if v_ref_60 >= 0:
            return self._sector_60_120(valpha, vbeta)
        elif v_ref_120 >= 0:
            return self._sector_120_180(valpha, vbeta)
        elif -valpha >= 0:
            return self._sector_180_240(valpha, vbeta)
        elif -v_ref_60 >= 0:
            return self._sector_240_300(valpha, vbeta)
        elif -v_ref_120 >= 0:
            return self._sector_300_360(valpha, vbeta)
        else:
            return self._sector_0_60(valpha, vbeta)
    
    def _add_common_injection(self, ta, tb, tc):
        """Add common-mode injection for centered PWM"""
        t0 = 1.0 - ta - tb - tc
        t_com = t0 * 0.5
        return ta + t_com, tb + t_com, tc + t_com
    
    def _sector_0_60(self, valpha, vbeta):
        t1 = valpha - vbeta * self.inv_sqrt3
        t2 = vbeta * 2.0 * self.inv_sqrt3
        return self._add_common_injection(t1 + t2, t2, 0.0)
    
    def _sector_60_120(self, valpha, vbeta):
        t1 = valpha + vbeta * self.inv_sqrt3
        t2 = -valpha + vbeta * self.inv_sqrt3
        return self._add_common_injection(t2, t1 + t2, 0.0)
    
    def _sector_120_180(self, valpha, vbeta):
        t1 = -valpha + vbeta * self.inv_sqrt3
        t2 = -valpha - vbeta * self.inv_sqrt3
        return self._add_common_injection(0.0, t1 + t2, t2)
    
    def _sector_180_240(self, valpha, vbeta):
        t1 = -valpha - vbeta * self.inv_sqrt3
        t2 = vbeta * 2.0 * self.inv_sqrt3
        return self._add_common_injection(0.0, t2, t1 + t2)
    
    def _sector_240_300(self, valpha, vbeta):
        t1 = -valpha + vbeta * self.inv_sqrt3
        t2 = -vbeta * 2.0 * self.inv_sqrt3
        return self._add_common_injection(t2, 0.0, t1 + t2)
    
    def _sector_300_360(self, valpha, vbeta):
        t1 = valpha + vbeta * self.inv_sqrt3
        t2 = -valpha - vbeta * self.inv_sqrt3
        return self._add_common_injection(t1 + t2, 0.0, t2)


# -------------------------------------------------------
# FOC Controller Class
# -------------------------------------------------------
class FieldOrientedController:
    """
    Field-Oriented Control implementation.
    Combines Clarke/Park transforms with PI controllers and SVPWM.
    """
    
    def __init__(self, vdc):
        self.vdc = vdc
        inv_sqrt3 = 0.577350269189626
        
        # PI controllers for d and q axis
        self.d_pid = PidController(0.0, 0.0, -inv_sqrt3, inv_sqrt3)
        self.q_pid = PidController(0.0, 0.0, -inv_sqrt3, inv_sqrt3)
        
        # SVPWM generator
        self.svpwm = SpaceVectorModulation()
        
        self.dt = 0.0001  # Will be set during simulation
        
    def set_tunings(self, d_kp, d_ki, q_kp, q_ki):
        """Set PID tunings (normalized by Vdc)"""
        self.d_pid.kp = d_kp / self.vdc
        self.d_pid.ki = d_ki / self.vdc
        self.q_pid.kp = q_kp / self.vdc
        self.q_pid.ki = q_ki / self.vdc
        
    def set_point(self, id_ref, iq_ref):
        """Set current references"""
        self.d_pid.set_point(id_ref)
        self.q_pid.set_point(iq_ref)
        
    def calculate(self, currents_abc, theta_e, dt):
        """
        Calculate duty cycles from measured currents and position.
        Matches C++ signature: Calculate(currents, position)
        """
        self.dt = dt
        ia, ib, ic = currents_abc
        
        # Clarke Transform: ABC -> αβ
        i_alpha = (2.0/3.0) * (ia - 0.5 * ib - 0.5 * ic)
        i_beta = (2.0/3.0) * (np.sqrt(3)/2 * ib - np.sqrt(3)/2 * ic)
        
        # Park Transform: αβ -> dq
        cos_theta = np.cos(theta_e)
        sin_theta = np.sin(theta_e)
        id_meas = i_alpha * cos_theta + i_beta * sin_theta
        iq_meas = -i_alpha * sin_theta + i_beta * cos_theta
        
        # PI control in dq frame
        vd = self.d_pid.process(id_meas, dt)
        vq = self.q_pid.process(iq_meas, dt)
        
        # Inverse Park Transform: dq -> αβ
        v_alpha = vd * cos_theta - vq * sin_theta
        v_beta = vd * sin_theta + vq * cos_theta
        
        # SVPWM: αβ -> duty cycles
        duty_a, duty_b, duty_c = self.svpwm.generate(v_alpha, v_beta)
        
        # Convert to percentage (0-100)
        return duty_a * 100.0, duty_b * 100.0, duty_c * 100.0


# -------------------------------------------------------
# Simulation Setup
# -------------------------------------------------------
def run_simulation():
    """Main simulation loop"""
    time_step = 0.0001  # 100 microseconds
    simulation_time = 0.2  # 200 milliseconds
    steps = int(simulation_time / time_step)
    
    # Create motor model
    params = PmsmModel.Parameters()
    model = PmsmModel(params, time_step)
    
    # Create FOC controller
    foc = FieldOrientedController(params.Vdc)
    
    # Set PID tunings (using discrete-time tuned values from C++)
    foc.set_tunings(0.15, 1.5, 0.15, 1.5)
    
    # Set current references
    foc.set_point(0.0, 1.0)  # Id=0, Iq=1.0A
    
    # Initial duty cycles
    duty_cycles = (50.0, 50.0, 50.0)
    
    # Data collection
    time_data = []
    ia_data = []
    ib_data = []
    ic_data = []
    theta_data = []
    
    # Simulation loop
    for i in range(steps):
        # Run motor model
        currents_abc, theta_e = model.run(duty_cycles)
        
        # Calculate new duty cycles
        duty_cycles = foc.calculate(currents_abc, theta_e, time_step)
        
        # Store data
        time_data.append(i * time_step)
        ia_data.append(currents_abc[0])
        ib_data.append(currents_abc[1])
        ic_data.append(currents_abc[2])
        theta_data.append(theta_e)
        
        # Debug print
        if i < 10 or i % 1000 == 0:
            print(f"Step {i}: ia={currents_abc[0]:6.3f} A, duty_a={int(duty_cycles[0])}, theta={theta_e:6.3f} rad")
    
    return time_data, ia_data, ib_data, ic_data, theta_data


# -------------------------------------------------------
# Main execution
# -------------------------------------------------------
t_vals, ia_vals, ib_vals, ic_vals, theta_e_vals = run_simulation()

# Convert to numpy arrays
iabc = np.column_stack([ia_vals, ib_vals, ic_vals])

# Save to CSV for validation
df = pd.DataFrame({
    "time_s": t_vals,
    "ia_A": ia_vals,
    "ib_A": ib_vals,
    "ic_A": ic_vals,
    "theta_e_rad": theta_e_vals
})
df.to_csv("pmsm_foc_measurements.csv", index=False)

# Print summary statistics
print("\n=== Simulation Summary ===")
print(f"Total simulation time: {t_vals[-1]:.3f} s")
print(f"Time steps: {len(t_vals)}")
print(f"Final electrical angle: {theta_e_vals[-1]:.3f} rad")
print(f"Final ia: {ia_vals[-1]:.3f} A")
print(f"Final ib: {ib_vals[-1]:.3f} A")
print(f"Final ic: {ic_vals[-1]:.3f} A")

# Analyze steady-state (last 20%)
steady_start = int(len(t_vals) * 0.8)
ia_ss = np.array(ia_vals[steady_start:])
ib_ss = np.array(ib_vals[steady_start:])
ic_ss = np.array(ic_vals[steady_start:])

ia_rms = np.sqrt(np.mean(ia_ss**2))
ib_rms = np.sqrt(np.mean(ib_ss**2))
ic_rms = np.sqrt(np.mean(ic_ss**2))

print("\n=== Steady-State Analysis (last 20%) ===")
print(f"Phase A RMS: {ia_rms:.3f} A")
print(f"Phase B RMS: {ib_rms:.3f} A")
print(f"Phase C RMS: {ic_rms:.3f} A")
print(f"Average RMS: {np.mean([ia_rms, ib_rms, ic_rms]):.3f} A")

# -------------------------------------------------------
# Plot results
# -------------------------------------------------------
plt.figure(figsize=(10, 8))

plt.subplot(2, 1, 1)
plt.plot(t_vals, ia_vals, label="i_a", linewidth=1.5)
plt.plot(t_vals, ib_vals, label="i_b", linewidth=1.5)
plt.plot(t_vals, ic_vals, label="i_c", linewidth=1.5)
plt.ylabel("Phase Currents [A]")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True, alpha=0.3)
plt.title("FOC Simulation Results - Phase Currents")

plt.subplot(2, 1, 2)
plt.plot(t_vals, theta_e_vals, linewidth=1.5)
plt.ylabel("Electrical Angle [rad]")
plt.xlabel("Time [s]")
plt.grid(True, alpha=0.3)
plt.title("Encoder Electrical Angle")

plt.tight_layout()
plt.savefig('pmsm_simulation_results.png', dpi=150)
print("\nPlot saved as 'pmsm_simulation_results.png'")

plt.show()
