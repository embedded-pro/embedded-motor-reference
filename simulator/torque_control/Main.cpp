#include "foc/interfaces/FieldOrientedController.hpp"
#include "simulator/pmsm/Model.hpp"
#include "simulator/torque_control/Plot.hpp"
#include "source/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "source/foc/instantiations/TrigonometricImpl.hpp"
#include "source/foc/interfaces/Driver.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sciplot/sciplot.hpp>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{
    class Simulator
    {
    public:
        void Run()
        {
            simulator::PmsmModel::Parameters params{};
            simulator::PmsmModel model{ params, std::chrono::duration_cast<std::chrono::duration<float>>(timeStep).count() };

            float Kp_current = 15.0f;
            float Ki_current = 2000.0f;
            float Kd_current = 0.0f;

            float Id_setpoint = 0.0f; // Field weakening current
            float Iq_setpoint = 5.0f; // Torque-producing current (A)

            std::cout << "Motor Parameters:\n";
            std::cout << "  R = " << params.R << " Ω\n";
            std::cout << "  L = " << params.Ld * 1000.0f << " mH\n";
            std::cout << "  Vdc = " << params.Vdc << " V\n";
            std::cout << "  Pole pairs = " << params.p << "\n";
            std::cout << "  Max current = 15 A\n";
            std::cout << "\nPID Tuning:\n";
            std::cout << "Current Loop (Id/Iq):\n";
            std::cout << "  Kp = " << Kp_current << "\n";
            std::cout << "  Ki = " << Ki_current << "\n";
            std::cout << "  Kd = " << Kd_current << "\n";
            std::cout << "  Output limit = ±1.0 (normalized modulation)\n";
            std::cout << "\nTorque Control Setpoint:\n";
            std::cout << "  Id = " << Id_setpoint << " A (field weakening)\n";
            std::cout << "  Iq = " << Iq_setpoint << " A (torque)\n\n";

            time.reserve(steps);
            i_a_data.reserve(steps);
            i_b_data.reserve(steps);
            i_c_data.reserve(steps);
            theta_data.reserve(steps);
            speed_data.reserve(steps);

            foc.SetPolePairs(static_cast<std::size_t>(params.p));
            foc.SetCurrentTunings(
                foc::Volts{ motorParameters.Vdc },
                foc::IdAndIqTunings{
                    { Kp_current, Ki_current, Kd_current },
                    { Kp_current, Ki_current, Kd_current } });
            foc.SetPoint(foc::IdAndIqPoint{ foc::Ampere{ Id_setpoint }, foc::Ampere{ Iq_setpoint } });

            std::cout << "Starting simulation...\n";
            std::cout << "Duration: " << simulationTime.count() << " ms\n";
            std::cout << "Time step: " << timeStep.count() << " μs\n";
            std::cout << "Total steps: " << steps << "\n";
            std::cout << "Motor: R=" << params.R << "Ω, L=" << params.Ld * 1000 << "mH, λ=" << params.psi_f << "Wb\n\n";

            for (size_t i = 0; i < steps; ++i)
            {
                auto [currentPhases, position] = model.Run(dutyCycles);
                dutyCycles = foc.Calculate(currentPhases, position);

                auto [i_a, i_b, i_c] = currentPhases;
                auto currentTime = static_cast<float>(i) * std::chrono::duration_cast<std::chrono::duration<float>>(timeStep).count();

                time.push_back(currentTime);
                i_a_data.push_back(i_a.Value());
                i_b_data.push_back(i_b.Value());
                i_c_data.push_back(i_c.Value());
                theta_data.push_back(position.Value());

                if (i > 0)
                {
                    float dt_sec = std::chrono::duration_cast<std::chrono::duration<float>>(timeStep).count();
                    float delta_theta = position.Value() - theta_data[i - 1];

                    if (delta_theta > M_PI)
                        delta_theta -= 2.0f * M_PI;
                    else if (delta_theta < -M_PI)
                        delta_theta += 2.0f * M_PI;

                    float omega_mech = delta_theta / dt_sec;
                    float speed_rpm = omega_mech * 60.0f / (2.0f * M_PI);
                    speed_data.push_back(speed_rpm);
                }
                else
                {
                    speed_data.push_back(0.0f);
                }

                if (i % (steps / 10) == 0)
                {
                    float progress = 100.0f * static_cast<float>(i) / static_cast<float>(steps);
                    std::cout << "Progress: " << std::fixed << std::setprecision(0) << progress << "%\n";
                }
            }

            std::cout << "\nSimulation completed!\n";

            graphics::PlotResults(time, i_a_data, i_b_data, i_c_data, theta_data);

            std::cout << "Plots saved to foc_simulation_results.png and foc_simulation_results.pdf\n";
        }

    private:
        static constexpr simulator::PmsmModel::Parameters motorParameters{};
        static constexpr std::chrono::microseconds timeStep{ 10 };
        static constexpr std::chrono::milliseconds simulationTime{ 1000 };
        static constexpr auto steps = static_cast<std::size_t>(std::chrono::duration_cast<std::chrono::duration<float>>(simulationTime).count() / std::chrono::duration_cast<std::chrono::duration<float>>(timeStep).count());

        std::vector<float> time;
        std::vector<float> i_a_data;
        std::vector<float> i_b_data;
        std::vector<float> i_c_data;
        std::vector<float> theta_data;
        std::vector<float> speed_data;

        foc::TrigonometricFunctions trigFunctions;
        foc::FieldOrientedControllerTorqueImpl foc{ trigFunctions };
        foc::PhasePwmDutyCycles dutyCycles{ hal::Percent(0), hal::Percent(0), hal::Percent(0) };
    };
}

int main()
{
    Simulator simulator;
    simulator.Run();

    return 0;
}
