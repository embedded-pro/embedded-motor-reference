#pragma once

#include "application/foc/interfaces/Driver.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"

namespace application
{
    class QuadratureEncoderDecorator
    {
    public:
        virtual foc::Radians Read() = 0;
    };

    template<typename Impl, typename = std::enable_if_t<std::is_base_of_v<hal::SynchronousQuadratureEncoder, Impl>>>
    class QuadratureEncoderDecoratorImpl
        : public QuadratureEncoderDecorator
    {
    public:
        template<typename... Args>
        QuadratureEncoderDecoratorImpl(uint32_t resolution, Args&&... args);

        foc::Radians Read() override;

    private:
        Impl encoder;
    };

    // Implementation

    template<typename Impl, typename Enable>
    template<typename... Args>
    QuadratureEncoderDecoratorImpl<Impl, Enable>::QuadratureEncoderDecoratorImpl(uint32_t resolution, Args&&... args)
        : encoder(std::forward<Args>(args)...)
    {
    }

    template<typename Impl, typename Enable>
    foc::Radians QuadratureEncoderDecoratorImpl<Impl, Enable>::Read()
    {
        static constexpr float twoPi = 2.0f * 3.14159265359f;

        auto count = static_cast<float>(encoder.Position());
        return foc::Radians{ count * twoPi / static_cast<float>(encoder.Resolution()) };
    }
}
