#pragma once

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
        StatusWithMessage Stop();
        StatusWithMessage ReadAdcWithSampleTime();
        StatusWithMessage SetPwmDuty(const infra::BoundedConstString& param);

    private:
        static constexpr std::size_t averageSampleSize = 10;
        static constexpr std::size_t numberOfChannels = 5;
        using AdcChannelSamples = infra::BoundedDeque<uint16_t>::WithMaxSize<averageSampleSize>;

        void StartAdc(HardwareFactory::SampleAndHold sampleAndHold);
        bool IsAdcBufferPopulated();

    private:
        const infra::BoundedVector<infra::BoundedConstString>::WithMaxSize<5> acceptedAdcValues{ { "shortest", "shorter", "medium", "longer", "longest" } };

        services::TerminalWithStorage& terminal;
        services::Tracer& tracer;
        infra::DelayedProxyCreator<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds, hal::Hertz)> pwmCreator;
        infra::DelayedProxyCreator<hal::AdcMultiChannel, void(HardwareFactory::SampleAndHold)> adcCreator;
        infra::DelayedProxyCreator<hal::SynchronousQuadratureEncoder, void()> encoderCreator;
        infra::BoundedVector<AdcChannelSamples>::WithMaxSize<numberOfChannels> adcChannelSamples;
    };
}
