#pragma once

#include "source/foc/implementations/ControllerBaseImpl.hpp"
#include "source/services/alignment/MotorAlignment.hpp"
#include "source/services/cli/TerminalBase.hpp"
#include "source/services/parameter_identification/MotorIdentification.hpp"
#include <type_traits>
#include <variant>

namespace application
{
    template<typename FocImpl, typename TerminalImpl, typename = std::enable_if_t<std::is_base_of_v<foc::ControllerBaseImpl, FocImpl>>>
    class MotorStateMachine
    {
    public:
        MotorStateMachine(services::TerminalFocBaseInteractor& terminal, foc::MotorDriver& driver, foc::Encoder& encoder);

    private:
        foc::MotorDriver& driver;
        foc::Encoder& encoder;
        std::variant<std::monostate, services::MotorAlignment, services::MotorIdentification, FocImpl> state;
    };

    // Implementation

    template<typename FocImpl, typename TerminalImpl, typename Enable>
    MotorStateMachine<FocImpl, TerminalImpl, Enable>::MotorStateMachine(services::TerminalFocBaseInteractor& terminal, foc::MotorDriver& driver, foc::Encoder& encoder)
        : driver(driver)
        , encoder(encoder)
    {
    }
}
