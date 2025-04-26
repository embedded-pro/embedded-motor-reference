#pragma once

#include "application/foc/FieldOrientedController.hpp"

namespace application
{
    class MotorFieldOrientedController
    {
    public:
        using IdAndIqTunnings = std::pair<controllers::Pid<float>::Tunnings, controllers::Pid<float>::Tunnings>;
        using IdAndIqPoint = std::pair<float, float>;

        MotorFieldOrientedController(MotorFieldOrientedControllerInterface& interface, FieldOrientedController& foc);

        void SetTunnings(IdAndIqTunnings tunnings);
        void SetPoint(const IdAndIqPoint& point);
        virtual void Enable();
        void Disable();
        bool IsRunning() const;

    protected:
        MotorFieldOrientedControllerInterface& interface;
        FieldOrientedController& foc;
        controllers::Pid<float> dPid;
        controllers::Pid<float> qPid;
        bool enabled = false;
    };
}
