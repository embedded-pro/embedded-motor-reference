#include "application/foc/FocWithTimer.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"

namespace application
{
    FocWithTimer::FocWithTimer(FocInput& input, FocOutput& output, Components& components)
        : input(input)
        , output(output)
        , spaceVectorModulator(components.spaceVectorModulator)
        , dPid(components.dPid)
        , qPid(components.qPid)
        , timer(components.timerId)
        , sampleTime(components.samplingTime)
        , park(components.trigonometricFunctions)
    {
        really_assert(components.timerId != infra::systemTimerServiceId);
    }

    void FocWithTimer::SetTunnings(IdAndIqTunnings tunnings)
    {
        dPid.SetTunnings(tunnings.first);
        qPid.SetTunnings(tunnings.second);
    }

    void FocWithTimer::SetTorque(float torque)
    {
        qPid.SetPoint(torque);
    }

    void FocWithTimer::Enable()
    {
        dPid.Enable();
        qPid.Enable();

        timer.Start(sampleTime, [this]()
            {
                auto& read = input.Read();

                auto idAndIq = park.Forward(clarke.Forward(read.first), read.second);
                auto twoPhaseVoltage = controllers::RotatingFrame<float>{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) };
                auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, read.second);
                auto pwmOutput = spaceVectorModulator.Generate(voltageAlphaBeta);

                output.Update(pwmOutput);
            });
    }

    void FocWithTimer::Disable()
    {
        timer.Cancel();
        output.Disable();
        dPid.Disable();
        qPid.Disable();
    }

    bool FocWithTimer::IsRunning() const
    {
        return timer.Armed();
    }
}
