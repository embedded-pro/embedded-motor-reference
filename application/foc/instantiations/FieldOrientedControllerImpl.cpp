#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"

namespace application
{
    FieldOrientedControllerImpl::FieldOrientedControllerImpl(math::TrigonometricFunctions<float>& trigFunctions)
        : park{ trigFunctions }
    {}

    std::tuple<Percent, Percent, Percent> FieldOrientedControllerImpl::Calculate(controllers::Pid<float>& dPid, controllers::Pid<float>& qPid, const std::tuple<MilliVolt, MilliVolt, MilliVolt>& voltagePhases, Degrees& position)
    {
        auto threePhaseCurrent = controllers::ThreePhase<float>{ std::get<0>(voltagePhases).Value(), std::get<1>(voltagePhases).Value(), std::get<2>(voltagePhases).Value() };
        auto angle = position.Value();
        auto idAndIq = park.Forward(clarke.Forward(threePhaseCurrent), angle);
        auto twoPhaseVoltage = controllers::RotatingFrame<float>{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) };
        auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, angle);
        auto output = spaceVectorModulator.Generate(voltageAlphaBeta);

        return std::tuple<Percent, Percent, Percent>{ output.a, output.b, output.c };
    }
}
