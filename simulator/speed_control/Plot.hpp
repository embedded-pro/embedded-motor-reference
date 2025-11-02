#pragma once

#include "sciplot/Plot.hpp"
#include <optional>
#include <sciplot/sciplot.hpp>
#include <string>
#include <vector>

namespace graphics
{
    void PlotResults(const std::vector<float>& time,
        const std::vector<float>& i_a,
        const std::vector<float>& i_b,
        const std::vector<float>& i_c,
        const std::vector<float>& theta);

    class Figure
    {
    public:
        struct Size
        {
            std::size_t Width;
            std::size_t Height;
        };

        struct Limits
        {
            float Min;
            float Max;
        };

        struct Axis
        {
            std::string Label;
            Limits Range;
        };

        struct PlotConfig
        {
            Axis XAxis;
            Axis YAxis;
        };

        struct DataPoint
        {
            std::vector<float> x;
            std::vector<float> y;
        };

        struct Plot
        {
            void Draw(const std::vector<float>& x, const std::vector<float>& y, const std::string& label, std::size_t lineWidth, const std::string& colour);
            sciplot::Plot plot;
        };

        Figure(const std::string& title, Size size);

        Plot& CreatePlot(PlotConfig config);
        void Save(const std::string& filename);

    private:
        std::string title;
        Size size;
        std::vector<Plot> plots;
    };
}
