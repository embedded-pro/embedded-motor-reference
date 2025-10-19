#pragma once

#include "application/foc/interfaces/Driver.hpp"
#include "application/foc/interfaces/FieldOrientedController.hpp"

namespace foc
{
    class ControllerBaseImpl
    {
    protected:
        ControllerBaseImpl(MotorDriver& interface, Encoder& position, FieldOrientedControllerBase& foc);

        void Enable();
        void Disable();
        bool IsRunning() const;

    private:
        MotorDriver& interface;
        Encoder& position;
        FieldOrientedControllerBase& foc;
        bool isRunning{ false };
    };
}
