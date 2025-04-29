#pragma once

#include "application/foc/FieldOrientedController.hpp"

namespace application
{
    class MotorFieldOrientedController
    {
    public:
        using IdAndIqTunnings = std::pair<controllers::Pid<float>::Tunnings, controllers::Pid<float>::Tunnings>;
        using IdAndIqPoint = std::pair<float, float>;

        virtual void SetTunnings(IdAndIqTunnings tunnings) = 0;
        virtual void SetPoint(const IdAndIqPoint& point) = 0;
        virtual void Enable() = 0;
        virtual void Disable() = 0;
        virtual bool IsRunning() const = 0;
    };

    class MotorFieldOrientedControllerImpl
        : public MotorFieldOrientedController
    {
    public:
        using IdAndIqTunnings = std::pair<controllers::Pid<float>::Tunnings, controllers::Pid<float>::Tunnings>;
        using IdAndIqPoint = std::pair<float, float>;

        MotorFieldOrientedControllerImpl(MotorFieldOrientedControllerInterface& interface, FieldOrientedController& foc);

        void SetTunnings(IdAndIqTunnings tunnings) override;
        void SetPoint(const IdAndIqPoint& point) override;
        void Enable() override;
        void Disable() override;
        bool IsRunning() const override;

    private:
        MotorFieldOrientedControllerInterface& interface;
        FieldOrientedController& foc;
        controllers::Pid<float> dPid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f }, true };
        controllers::Pid<float> qPid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f }, true };
        bool enabled = false;
    };
}
