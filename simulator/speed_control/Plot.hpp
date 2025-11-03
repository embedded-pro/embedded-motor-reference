#pragma once

#include <sciplot/sciplot.hpp>
#include <vector>

namespace graphics
{
    void PlotResults(const std::vector<float>& time,
        const std::vector<float>& i_a,
        const std::vector<float>& i_b,
        const std::vector<float>& i_c,
        const std::vector<float>& theta);
}
