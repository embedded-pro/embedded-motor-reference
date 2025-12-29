#include "Plot.hpp"

namespace graphics
{
    void PlotResults(const std::vector<float>& time,
        const std::vector<float>& i_a,
        const std::vector<float>& i_b,
        const std::vector<float>& i_c,
        const std::vector<float>& theta)
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
        plot1.yrange(-15.0, 10.0);
        plot1.legend()
            .atOutsideRight()
            .displayHorizontal()
            .displayExpandWidthBy(2);

        plot1.drawCurve(time_d, i_a_d).label("i_a").lineWidth(2).lineColor("blue");
        plot1.drawCurve(time_d, i_b_d).label("i_b").lineWidth(2).lineColor("orange");
        plot1.drawCurve(time_d, i_c_d).label("i_c").lineWidth(2).lineColor("green");

        sciplot::Plot plot2;
        plot2.xlabel("Time [s]");
        plot2.ylabel("Electrical Angle [rad]");
        plot2.xrange(0.0, time.back());
        plot2.yrange(0.0, 7.0);

        plot2.drawCurve(time_d, theta_d).label("theta").lineWidth(2).lineColor("blue");

        // Plot 3: Zoomed-in view of phase currents to show 120-degree phase shift
        sciplot::Plot plot3;
        plot3.xlabel("Time [s]");
        plot3.ylabel("Phase Currents [A]");
        // Zoom to show approximately 2-3 electrical cycles during steady state
        // Show last 200ms of simulation (steady state)
        double zoom_end = static_cast<double>(time.back());
        double zoom_start = std::max(zoom_end - 0.2, 0.0); // 200ms window at the end
        plot3.xrange(zoom_start, zoom_end);
        plot3.yrange(-0.3, 0.3);
        plot3.legend()
            .atOutsideRight()
            .displayHorizontal()
            .displayExpandWidthBy(2);

        plot3.drawCurve(time_d, i_a_d).label("i_a").lineWidth(2).lineColor("blue");
        plot3.drawCurve(time_d, i_b_d).label("i_b").lineWidth(2).lineColor("orange");
        plot3.drawCurve(time_d, i_c_d).label("i_c").lineWidth(2).lineColor("green");

        sciplot::Figure fig = { { plot1 }, { plot2 }, { plot3 } };
        fig.title("FOC Torque Simulation Results");
        fig.size(950, 1200); // Increased height for third plot

        fig.save("foc_torque_simulation_results.png");
    }
}
