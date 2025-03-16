#ifndef APPLICATION_FOC_WITH_TIMER_HPP
#define APPLICATION_FOC_WITH_TIMER_HPP

#include "application/pid/PidWithTimer.hpp"
#include "infra/timer/Timer.hpp"
#include "infra/util/BoundedVector.hpp"
#include "infra/util/WithStorage.hpp"
#include "numerical/controllers/SpaceVectorModulation.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"
#include "numerical/math/TrigonometricFunctions.hpp"

namespace application
{
    class PerformanceTracker
    {
    public:
        virtual void Start() = 0;
        virtual uint32_t ElapsedCycles() = 0;
    };

    class FocInput
    {
    public:
        using Input = std::pair<controllers::ThreePhase<float>, float>;
        virtual Input& Read() = 0;
    };

    class FocOutput
    {
    public:
        using Output = controllers::SpaceVectorModulation<float>::Output;

        virtual void Update(Output& pwm) = 0;
        virtual void Disable() = 0;
    };

    class SpaceVectorModulator
    {
    public:
        using TwoVoltagePhase = controllers::TwoPhase<float>;

        virtual FocOutput::Output Generate(TwoVoltagePhase& voltagePhase) = 0;
    };

    class FocWithTimer
    {
    public:
        struct Components
        {
            math::TrigonometricFunctions<float>& trigonometricFunctions;
            SpaceVectorModulator& spaceVectorModulator;
            Pid& dPid;
            Pid& qPid;
            infra::Duration samplingTime;
            uint32_t timerId;
        };

        using IdAndIqTunnings = std::pair<controllers::Pid<float>::Tunnings, controllers::Pid<float>::Tunnings>;

        FocWithTimer(FocInput& input, FocOutput& output, Components& components);

        void SetTunnings(IdAndIqTunnings tunnings);
        void SetTorque(float torque);
        virtual void Enable();
        void Disable();
        bool IsRunning() const;

    protected:
        FocInput& input;
        FocOutput& output;
        SpaceVectorModulator& spaceVectorModulator;
        Pid& dPid;
        Pid& qPid;
        infra::TimerRepeating timer;
        infra::Duration sampleTime;
        controllers::Clarke<float> clarke;
        controllers::Park<float> park;
    };

    class FocWithTimerAndProfiler
        : public FocWithTimer
    {
    public:
        enum class Stage
        {
            input,
            directClakeAndPark,
            dPid,
            qPid,
            inversePark,
            spaceVectorModulator,
            output,
            total,
        };

        struct ExecutionTimes
        {
            Stage stage;
            uint32_t elapsedCycles;
        };

        template<std::size_t Size>
        using WithProfilerSize = infra::WithStorage<FocWithTimerAndProfiler, infra::BoundedVector<ExecutionTimes>::WithMaxSize<Size>>;

        FocWithTimerAndProfiler(infra::BoundedVector<ExecutionTimes>& executionTimes, FocInput& input, FocOutput& output, Components& components, PerformanceTracker& profiler);

        void Enable() override;
        infra::BoundedVector<ExecutionTimes>& ProfileData() const;

    private:
        infra::BoundedVector<ExecutionTimes>& executionTimes;
        PerformanceTracker& profiler;
    };
}

#endif
