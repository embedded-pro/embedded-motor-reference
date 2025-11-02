#pragma once

#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "application/foc/instantiations/TrigonometricImpl.hpp"
#include "application/foc/interfaces/Driver.hpp"
#include "application/hardware/HardwareFactory.hpp"
#include "hal/interfaces/AdcMultiChannel.hpp"
#include "infra/util/BoundedDeque.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace application
{
    class TerminalInteractor
    {
    public:
        TerminalInteractor(services::TerminalWithStorage& terminal, application::HardwareFactory& hardware);

    private:
        using StatusWithMessage = services::TerminalWithStorage::StatusWithMessage;

        void PrintHeader();
        StatusWithMessage ConfigurePwm(const infra::BoundedConstString& param);
        StatusWithMessage ConfigureAdc(const infra::BoundedConstString& param);
        StatusWithMessage SimulateFoc(const infra::BoundedConstString& param);
        StatusWithMessage ConfigurePid(const infra::BoundedConstString& param);
        void RunFocSimulation(std::tuple<foc::Ampere, foc::Ampere, foc::Ampere, foc::Radians> input);
        StatusWithMessage Stop();
        StatusWithMessage ReadAdcWithSampleTime();
        StatusWithMessage SetPwmDuty(const infra::BoundedConstString& param);

    private:
        static constexpr std::size_t averageSampleSize = 10;
        static constexpr std::size_t numberOfChannels = 5;
        using AdcChannelSamples = infra::BoundedDeque<uint16_t>::WithMaxSize<averageSampleSize>;

        void StartAdc(HardwareFactory::SampleAndHold sampleAndHold);
        bool IsAdcBufferPopulated() const;

    private:
        const infra::BoundedVector<infra::BoundedConstString>::WithMaxSize<5> acceptedAdcValues{ { "shortest", "shorter", "medium", "longer", "longest" } };

        services::TerminalWithStorage& terminal;
        services::Tracer& tracer;
        infra::DelayedProxyCreator<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds, hal::Hertz)> pwmCreator;
        infra::DelayedProxyCreator<hal::AdcMultiChannel, void(HardwareFactory::SampleAndHold)> adcCreator;
        infra::DelayedProxyCreator<hal::SynchronousQuadratureEncoder, void()> encoderCreator;
        infra::BoundedVector<AdcChannelSamples>::WithMaxSize<numberOfChannels> adcChannelSamples;
        hal::PerformanceTracker& performanceTimer;
        foc::Volts Vdc;
        controllers::PidTunings<float> dPidTunings;
        controllers::PidTunings<float> qPidTunings;
        foc::TrigonometricFunctions trigFunctions;
        foc::FieldOrientedControllerSpeedImpl foc;
    };
}
