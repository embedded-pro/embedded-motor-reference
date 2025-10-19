#pragma once

#include "application/foc/implementations/ControllerBaseImpl.hpp"
#include "application/foc/interfaces/Controller.hpp"
#include "application/foc/interfaces/FieldOrientedController.hpp"

namespace foc
{
    class SpeedControllerImpl
        : public FocMotorSpeedController
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
