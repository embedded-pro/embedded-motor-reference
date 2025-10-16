#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "application/foc/MotorFieldOrientedControllerInterface.hpp"

namespace application
{
    FieldOrientedControllerImpl::FieldOrientedControllerImpl(math::TrigonometricFunctions<float>& trigFunctions)
        : park{ trigFunctions }
    {}

    std::tuple<hal::Percent, hal::Percent, hal::Percent> FieldOrientedControllerImpl::Calculate(controllers::Pid<float>& dPid, controllers::Pid<float>& qPid, const std::tuple<MilliAmpere, MilliAmpere, MilliAmpere>& currentPhases, Degrees& position)
    {
        auto threePhaseCurrent = controllers::ThreePhase<float>{ std::get<0>(currentPhases).Value(), std::get<1>(currentPhases).Value(), std::get<2>(currentPhases).Value() };
        auto angle = position.Value();
        auto idAndIq = park.Forward(clarke.Forward(threePhaseCurrent), angle);
        auto twoPhaseVoltage = controllers::RotatingFrame<float>{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) };
        auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, angle);
        auto output = spaceVectorModulator.Generate(voltageAlphaBeta);

        return std::tuple<hal::Percent, hal::Percent, hal::Percent>{ hal::Percent(static_cast<uint8_t>(output.a)), hal::Percent(static_cast<uint8_t>(output.b)), hal::Percent(static_cast<uint8_t>(output.c)) };
    }
}
