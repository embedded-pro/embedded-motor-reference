#pragma once

#include "application/foc/FieldOrientedController.hpp"
#include <gmock/gmock.h>

namespace application
{
    class FieldOrientedControllerMock
        : public FieldOrientedController
    {
    public:
        MOCK_METHOD((std::tuple<Percent, Percent, Percent>), Calculate, ((controllers::Pid<float>&), (controllers::Pid<float>&), (const std::tuple<MilliVolt, MilliVolt, MilliVolt>&), (Degrees&)), (override));
    };
}
