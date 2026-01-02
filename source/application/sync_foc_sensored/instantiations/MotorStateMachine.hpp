#pragma once

#include "foc/instantiations/TrigonometricImpl.hpp"
#include "source/foc/implementations/ControllerBaseImpl.hpp"
#include "source/foc/interfaces/FieldOrientedController.hpp"
#include "source/services/alignment/MotorAlignmentImpl.hpp"
#include "source/services/cli/TerminalBase.hpp"
#include "source/services/parameter_identification/MotorIdentificationImpl.hpp"
#include <type_traits>
#include <variant>

namespace application
{
    struct TerminalAndTracer
    {
        services::TerminalWithStorage& terminal;
        services::Tracer& tracer;
    };

    struct MotorDriverAndEncoder
    {
        foc::MotorDriver& driver;
        foc::Encoder& encoder;
    };

    template<typename FocImpl, typename ControllerImpl, typename TerminalImpl>
    constexpr bool IsValidMotorStateMachineTypes_v =
        std::is_base_of_v<foc::FieldOrientedControllerBase, FocImpl> &&
        std::is_base_of_v<foc::ControllerBaseImpl, ControllerImpl> &&
        std::is_base_of_v<services::TerminalFocBaseInteractor, TerminalImpl>;

    template<typename FocImpl, typename ControllerImpl, typename TerminalImpl, typename = std::enable_if_t<IsValidMotorStateMachineTypes_v<FocImpl, ControllerImpl, TerminalImpl>>>
    class MotorStateMachine
    {
    public:
        template<typename... FocArgs>
        MotorStateMachine(const TerminalAndTracer& terminalAndTracer, const MotorDriverAndEncoder& motorDriverAndEncoder, foc::Volts vdc, FocArgs&&... focArgs);

    private:
        services::TerminalWithStorage& terminal;
        services::Tracer& tracer;
        foc::MotorDriver& driver;
        foc::Encoder& encoder;
        foc::Volts vdc;
        foc::TrigonometricFunctions trigonometricFunctions;
        FocImpl focImpl;
        std::variant<std::monostate, services::MotorAlignmentImpl, services::MotorIdentificationImpl, ControllerImpl> motorStates;
        std::variant<std::monostate, TerminalImpl> terminalStates;
    };

    // Implementation

    template<typename FocImpl, typename ControllerImpl, typename TerminalImpl, typename Enable>
    template<typename... FocArgs>
    MotorStateMachine<FocImpl, ControllerImpl, TerminalImpl, Enable>::MotorStateMachine(const TerminalAndTracer& terminalAndTracer, const MotorDriverAndEncoder& motorDriverAndEncoder, foc::Volts vdc, FocArgs&&... focArgs)
        : terminal(terminalAndTracer.terminal)
        , tracer(terminalAndTracer.tracer)
        , driver(motorDriverAndEncoder.driver)
        , encoder(motorDriverAndEncoder.encoder)
        , vdc(vdc)
        , focImpl{ trigonometricFunctions, std::forward<FocArgs>(focArgs)... }
    {
        terminal.AddCommand({ { "ident_par", "ip", "Identify Parameters, which are resistance, inductance and number of pole pairs." },
            [this](const auto& params)
            {
                motorStates.template emplace<services::MotorAlignmentImpl>(this->driver, this->encoder, this->vdc);
            } });

        terminal.AddCommand({ { "align_motor", "am", "Align Motor." },
            [this](const auto& params)
            {
                motorStates.template emplace<services::MotorIdentificationImpl>(this->driver, this->encoder, this->vdc);
            } });

        terminal.AddCommand({ { "foc", "foc", "Start FOC controller." },
            [this](const auto& params)
            {
                this->focImpl.Reset();

                motorStates.template emplace<ControllerImpl>(this->driver, this->encoder, this->focImpl);
                auto focController = std::get<ControllerImpl>(this->motorStates);
                terminalStates.template emplace<TerminalImpl>(this->terminal, this->vdc, focController, focController);
            } });
    }
}
