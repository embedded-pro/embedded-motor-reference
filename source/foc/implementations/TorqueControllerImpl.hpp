#pragma once

#include "source/foc/implementations/ControllerBaseImpl.hpp"
#include "source/foc/interfaces/Controller.hpp"

namespace foc
{
    class TorqueControllerImpl
        : public TorqueController
        , private ControllerBaseImpl
    {
    public:
        TorqueControllerImpl(MotorDriver& interface, Encoder& position, FieldOrientedControllerTorqueControl& foc);

        void SetCurrentTunings(Volts Vcd, IdAndIqTunings tunings) override;
        void SetPoint(const IdAndIqPoint& point) override;
        void Enable() override;
        void Disable() override;
        bool IsRunning() const override;
        hal::Hertz BaseFrequency() const override;

    private:
        FieldOrientedControllerTorqueControl& foc;
    };
}
