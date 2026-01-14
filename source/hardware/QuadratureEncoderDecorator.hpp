#pragma once

#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include "source/foc/interfaces/Units.hpp"
#include <cmath>
#include <numbers>

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
        explicit QuadratureEncoderDecoratorImpl(uint32_t resolution, Args&&... args);

        foc::Radians Read() override;

    private:
        Impl encoder;
    };

    // Implementation

    template<typename Impl, typename Enable>
    template<typename... Args>
    QuadratureEncoderDecoratorImpl<Impl, Enable>::QuadratureEncoderDecoratorImpl(uint32_t, Args&&... args)
        : encoder(std::forward<Args>(args)...)
    {
    }

    template<typename Impl, typename Enable>
    foc::Radians QuadratureEncoderDecoratorImpl<Impl, Enable>::Read()
    {
        static constexpr float twoPi = 2.0f * std::numbers::pi_v<float>;
        static constexpr float pi = std::numbers::pi_v<float>;

        // Cache encoder values to avoid redundant calls/conversions
        const float count = static_cast<float>(encoder.Position());
        const float resolution = static_cast<float>(encoder.Resolution());
        const float angle = count * twoPi / resolution;
        
        // Wrap angle to [-pi, pi]
        const float angle_plus_pi = angle + pi;
        const float wrapped = angle - twoPi * std::floor(angle_plus_pi / twoPi);

        return foc::Radians{ wrapped };
    }
}
