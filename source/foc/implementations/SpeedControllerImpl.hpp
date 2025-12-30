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

        void SetCurrentTunings(Volts Vcd, IdAndIqTunings tunings) override;
        void SetSpeedTunings(Volts Vcd, const SpeedTunings& speedTuning) override;
        void SetPoint(RadiansPerSecond point) override;
        void Enable() override;
        void Disable() override;
        bool IsRunning() const override;
        hal::Hertz BaseFrequency() const override;

    private:
        FieldOrientedControllerSpeedControl& foc;
    };
}
