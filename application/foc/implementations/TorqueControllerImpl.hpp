#pragma once

#include "application/foc/implementations/ControllerBaseImpl.hpp"
#include "application/foc/interfaces/Controller.hpp"

namespace foc
{
    class TorqueControllerImpl
        : public TorqueController
        , private ControllerBaseImpl
    {
    public:
        TorqueControllerImpl(MotorDriver& interface, Encoder& position, FieldOrientedControllerTorqueControl& foc);

        void SetTunings(Volts Vcd, IdAndIqTunings tunings) override;
        void SetPoint(const IdAndIqPoint& point) override;
        void Enable() override;
        void Disable() override;
        bool IsRunning() const override;

    private:
        FieldOrientedControllerTorqueControl& foc;
    };
}
