#pragma once

#include "source/foc/implementations/ControllerBaseImpl.hpp"
#include "source/foc/interfaces/Controller.hpp"
#include "source/foc/interfaces/FieldOrientedController.hpp"

namespace foc
{
    class SpeedControllerImpl
        : public SpeedController
        , private ControllerBaseImpl
    {
    public:
        SpeedControllerImpl(MotorDriver& interface, Encoder& position, FieldOrientedControllerSpeedControl& foc);

        void SetTunings(Volts Vcd, const SpeedTunings& speedTuning, const IdAndIqTunings& torqueTunings) override;
        void SetPoint(RadiansPerSecond point) override;
        void Enable() override;
        void Disable() override;
        bool IsRunning() const override;

    private:
        FieldOrientedControllerSpeedControl& foc;
    };
}
