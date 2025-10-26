#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "application/foc/instantiations/TrigonometricImpl.hpp"
#include "application/foc/interfaces/Driver.hpp"
#include "foc/interfaces/FieldOrientedController.hpp"
#include "simulator/pmsm/Model.hpp"
#include <cmath>
#include <iostream>
#include <sciplot/sciplot.hpp>
#include <utility>
#include <vector>

namespace
{
    class Simulator
    {
    public:
        void Run()
        {
            float timeStep = 0.0001f;
            float simulationTime = 0.2f;

            simulator::PmsmModel::Parameters params{};
            simulator::PmsmModel model{ params, timeStep };
            auto steps = static_cast<size_t>(simulationTime / timeStep);

            // PID Tuning for current control
            // Calibrated gains for the velocity-form PID with anti-windup
            // Based on motor parameters: R=0.86Ω, L=0.365mH, Vdc=48V
            // Target: Fast response with minimal overshoot

            float Kp = 0.15f; // Proportional gain [V/A] - provides fast response
            float Ki = 1.5f;  // Integral gain [V/(A*s)] - eliminates steady-state error
            float Kd = 0.0f;  // Derivative gain [V*s/A] - not needed for current control

            std::cout << "Motor Parameters:\n";
            std::cout << "  R = " << params.R << " Ω\n";
            std::cout << "  L = " << params.Ld * 1000.0f << " mH\n";
            std::cout << "  Vdc = " << params.Vdc << " V\n";
            std::cout << "\nPID Tuning (before normalization):\n";
            std::cout << "  Kp = " << Kp << " V/A\n";
            std::cout << "  Ki = " << Ki << " V/(A*s)\n";
            std::cout << "  Ki_discrete = " << Ki * timeStep << "\n\n";

            foc.SetTunings(
                foc::Volts{ motorParameters.Vdc },
                foc::SpeedTunings{ 0.0f, 0.0f, 0.0f },
                foc::IdAndIqTunings{ { Kp, Ki * timeStep, Kd }, { Kp, Ki * timeStep, Kd } });

            foc.SetPoint(foc::RadiansPerSecond{ 1.0f });

            std::vector<float> time;
            std::vector<float> i_a_data;
            std::vector<float> i_b_data;
            std::vector<float> i_c_data;
            std::vector<float> theta_data;

            time.reserve(steps);
            i_a_data.reserve(steps);
            i_b_data.reserve(steps);
            i_c_data.reserve(steps);
            theta_data.reserve(steps);

            for (size_t i = 0; i < steps; ++i)
            {
                auto [currentPhases, position] = model.Run(dutyCycles);
                dutyCycles = foc.Calculate(currentPhases, position);

                auto [i_a, i_b, i_c] = currentPhases;
                time.push_back(static_cast<float>(i) * timeStep);
                i_a_data.push_back(i_a.Value());
                i_b_data.push_back(i_b.Value());
                i_c_data.push_back(i_c.Value());
                theta_data.push_back(position.Value());

                // Debug print first few steps
                if (i < 10 || i % 100 == 0)
                {
                    auto [da, db, dc] = dutyCycles;
                    std::cout << "Step " << i << ": i_a=" << i_a.Value()
                              << " duty_a=" << static_cast<int>(da.Value())
                              << " theta=" << position.Value() << "\n";
                }
            }

            PlotResults(time, i_a_data, i_b_data, i_c_data, theta_data);
        }

    private:
        void PlotResults(const std::vector<float>& time,
            const std::vector<float>& i_a,
            const std::vector<float>& i_b,
            const std::vector<float>& i_c,
            const std::vector<float>& theta) const
        {
            using namespace sciplot;

            std::vector<double> time_d(time.begin(), time.end());
            std::vector<double> i_a_d(i_a.begin(), i_a.end());
            std::vector<double> i_b_d(i_b.begin(), i_b.end());
            std::vector<double> i_c_d(i_c.begin(), i_c.end());
            std::vector<double> theta_d(theta.begin(), theta.end());

            Plot plot1;
            plot1.xlabel("Time [s]");
            plot1.ylabel("Phase Currents [A]");
            plot1.xrange(0.0, time.back());
            plot1.yrange(-30.0, 30.0);
            plot1.legend()
                .atOutsideRight()
                .displayHorizontal()
                .displayExpandWidthBy(2);

            plot1.drawCurve(time_d, i_a_d).label("i_a").lineWidth(2).lineColor("blue");
            plot1.drawCurve(time_d, i_b_d).label("i_b").lineWidth(2).lineColor("orange");
            plot1.drawCurve(time_d, i_c_d).label("i_c").lineWidth(2).lineColor("green");

            Plot plot2;
            plot2.xlabel("Time [s]");
            plot2.ylabel("Electrical Angle [rad]");
            plot2.xrange(0.0, time.back());
            plot2.yrange(0.0, 7.0);

            plot2.drawCurve(time_d, theta_d).label("theta").lineWidth(2).lineColor("blue");

            Figure fig = { { plot1 }, { plot2 } };
            fig.title("FOC Simulation Results");
            fig.size(950, 800);

            fig.save("foc_simulation_results.png");
            fig.save("foc_simulation_results.pdf");

            std::cout << "Plots saved to foc_simulation_results.png and foc_simulation_results.pdf\n";
        }

        static constexpr simulator::PmsmModel::Parameters motorParameters{};

        foc::TrigonometricFunctions trigFunctions;
        foc::FieldOrientedControllerSpeedImpl foc{ trigFunctions };
        std::tuple<hal::Percent, hal::Percent, hal::Percent> dutyCycles{ hal::Percent(0), hal::Percent(0), hal::Percent(0) };
    };

}

int main()

{
    Simulator simulator;
    simulator.Run();

    return 0;
}
