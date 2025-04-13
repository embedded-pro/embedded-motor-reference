#ifndef APPLICATION_BLDC_LOGIC_MOTOR_CONTROLLER_IMPL_HPP
#define APPLICATION_BLDC_LOGIC_MOTOR_CONTROLLER_IMPL_HPP

#include "application/foc/FocWithTimer.hpp"
#include "application/foc/instantiations/SpaceVectorModulatorImpl.hpp"
#include "application/motors/BLDC/components/MotorController.hpp"
#include "application/motors/BLDC/components/TrigonometricImpl.hpp"
#include "application/pid/PidWithTimer.hpp"
#include "application/pid/instantiations/PidImpl.hpp"
#include "hal/synchronous_interfaces/SynchronousAdc.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"

namespace application
{
    class FocControllerImpl
        : public FocController
        , private FocInput
        , private FocOutput
    {
    public:
        struct Input
        {
            hal::SynchronousAdc& phaseA;
            hal::SynchronousAdc& phaseB;
            hal::SynchronousQuadratureEncoder& theta;
        };

        FocControllerImpl(Input& input, hal::SynchronousThreeChannelsPwm& output, FocWithTimer::Components& components);

        // Implementation of MotorController
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetDQPidParameters(const std::pair<PidFocParameters, PidFocParameters>& pidDAndQParameters) override;
        void SetTorque(const Torque& torque) override;
        void Start() override;
        void Stop() override;

        // Implementation of FocInput
        FocInput::Input Read() override;

        // Implementation of FocOutput
        void Update(Output& pwm) override;
        void Disable() override;

    private:
        Input& input;
        hal::SynchronousThreeChannelsPwm& output;
        FocWithTimer foc;
        FocWithTimer::IdAndIqPoint focSetPoint;
        FocWithTimer::IdAndIqTunnings idAndIqTunnings{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };
    };
}

#endif
