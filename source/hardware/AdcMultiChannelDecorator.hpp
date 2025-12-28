#pragma once

#include "hal/interfaces/AdcMultiChannel.hpp"
#include "source/foc/interfaces/Driver.hpp"
#include <type_traits>
#include <utility>

namespace application
{
    class AdcMultiChannelDecorator
    {
    public:
        virtual void Measure(const infra::Function<void(foc::Ampere phaseA, foc::Ampere phaseB, foc::Ampere phaseC)>& onDone) = 0;
        virtual void Stop() = 0;
    };

    template<typename Impl, typename = std::enable_if_t<std::is_base_of_v<hal::AdcMultiChannel, Impl>>>
    class AdcMultiChannelDecoratorImpl
        : public AdcMultiChannelDecorator
    {
    public:
        template<typename... Args>
        AdcMultiChannelDecoratorImpl(float adcToAmpereFactor, Args&&... args);

        void Measure(const infra::Function<void(foc::Ampere phaseA, foc::Ampere phaseB, foc::Ampere phaseC)>& onDone) override;
        void Stop() override;

    private:
        Impl adc;
        float adcToAmpereFactor = 0.0f;
        infra::Function<void(foc::Ampere phaseA, foc::Ampere phaseB, foc::Ampere phaseC)> onMeasurementDone;
    };

    // Implementation

    template<typename Impl, typename Enable>
    template<typename... Args>
    AdcMultiChannelDecoratorImpl<Impl, Enable>::AdcMultiChannelDecoratorImpl(float adcToAmpereFactor, Args&&... args)
        : adc(std::forward<Args>(args)...)
        , adcToAmpereFactor(adcToAmpereFactor)
    {
    }

    template<typename Impl, typename Enable>
    void AdcMultiChannelDecoratorImpl<Impl, Enable>::Measure(const infra::Function<void(foc::Ampere phaseA, foc::Ampere phaseB, foc::Ampere phaseC)>& onDone)
    {
        onMeasurementDone = onDone;
        adc.Measure([this](auto samples)
            {
                onMeasurementDone(foc::Ampere{ static_cast<float>(samples[0]) * adcToAmpereFactor }, foc::Ampere{ static_cast<float>(samples[1]) * adcToAmpereFactor }, foc::Ampere{ static_cast<float>(samples[2]) * adcToAmpereFactor });
            });
    }

    template<typename Impl, typename Enable>
    void AdcMultiChannelDecoratorImpl<Impl, Enable>::Stop()
    {
        adc.Stop();
    }
}
