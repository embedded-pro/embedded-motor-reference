#pragma once

#include "source/foc/interfaces/Driver.hpp"
#include "source/foc/interfaces/FieldOrientedController.hpp"

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
