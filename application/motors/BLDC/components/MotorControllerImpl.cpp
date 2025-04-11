#include "application/motors/BLDC/components/MotorControllerImpl.hpp"

namespace application
{
    FocControllerImpl::FocControllerImpl(Input& input, hal::SynchronousThreeChannelsPwm& output, FocWithTimer::Components& components)
        : input{ input }
        , output{ output }
        , foc{ *this, *this, components }
    {}

    void FocControllerImpl::AutoTune(const infra::Function<void()>& onDone)
    {
    }

    void FocControllerImpl::SetDQPidParameters(const std::pair<PidFocParameters, PidFocParameters>& pidDAndQParameters)
    {
        if (pidDAndQParameters.first.kp)
            idAndIqTunnings.first.kp = *pidDAndQParameters.first.kp;

        if (pidDAndQParameters.first.ki)
            idAndIqTunnings.first.ki = *pidDAndQParameters.first.ki;

        if (pidDAndQParameters.first.kd)
            idAndIqTunnings.first.kd = *pidDAndQParameters.first.kd;

        if (pidDAndQParameters.second.kp)
            idAndIqTunnings.second.kp = *pidDAndQParameters.second.kp;

        if (pidDAndQParameters.second.ki)
            idAndIqTunnings.second.ki = *pidDAndQParameters.second.ki;

        if (pidDAndQParameters.second.kd)
            idAndIqTunnings.second.kd = *pidDAndQParameters.second.kd;

        foc.SetTunnings(idAndIqTunnings);
    }

    void FocControllerImpl::SetSpeedPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd)
    {
    }

    void FocControllerImpl::SetPositionPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd)
    {
    }

    void FocControllerImpl::SetTorque(const Torque& speed)
    {
        focSetPoint.first = speed.Value();
        focSetPoint.second = 0.0;

        foc.SetPoint(focSetPoint);
    }

    void FocControllerImpl::SetSpeed(const RevPerMinute& speed)
    {
    }

    void FocControllerImpl::SetPosition(const Degrees& position)
    {
    }

    void FocControllerImpl::Start()
    {
        foc.Enable();
    }

    void FocControllerImpl::Stop()
    {
        foc.Disable();
    }

    void FocControllerImpl::EnableTorqueControl()
    {
    }

    void FocControllerImpl::EnableSpeedControl()
    {
    }

    void FocControllerImpl::EnablePositionControl()
    {
    }

    FocControllerImpl::FocInput::Input FocControllerImpl::Read()
    {
        auto a = static_cast<float>(input.phaseA.Measure(1)[0] * 3300);
        auto b = static_cast<float>(input.phaseB.Measure(1)[0] * 3300);
        auto c = -(a + b);
        auto theta = input.theta.Position() / input.theta.Resolution();

        return std::make_pair<controllers::ThreePhase<float>, float>({ a, b, c }, theta);
    }

    void FocControllerImpl::Update(Output& pwm)
    {
        auto a = hal::Percent(static_cast<uint8_t>(pwm.a * 100.0f));
        auto b = hal::Percent(static_cast<uint8_t>(pwm.b * 100.0f));
        auto c = hal::Percent(static_cast<uint8_t>(pwm.c * 100.0f));
        output.Start(a, b, c);
    }

    void FocControllerImpl::Disable()
    {
        output.Stop();
    }
}
