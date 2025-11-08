#include "application/foc/implementations/ControllerBaseImpl.hpp"

namespace foc
{
    ControllerBaseImpl::ControllerBaseImpl(MotorDriver& interface, Encoder& position, FieldOrientedControllerBase& foc)
        : interface{ interface }
        , position{ position }
        , foc{ foc }
    {
        interface.PhaseCurrentsReady(hal::Hertz{ 10000 }, [this](auto currentPhases)
            {
                auto positionValue = this->position.Read();
                this->interface.ThreePhasePwmOutput(this->foc.Calculate(currentPhases, positionValue));
            });
    }

    void ControllerBaseImpl::Enable()
    {
        isRunning = true;
        foc.Reset();
        interface.Start();
    }

    void ControllerBaseImpl::Disable()
    {
        isRunning = false;
        interface.Stop();
    }

    bool ControllerBaseImpl::IsRunning() const
    {
        return isRunning;
    }
}
