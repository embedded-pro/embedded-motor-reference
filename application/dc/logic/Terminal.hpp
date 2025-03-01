#ifndef APPLICATION_DC_LOGIC_TERMINAL_HPP
#define APPLICATION_DC_LOGIC_TERMINAL_HPP

#include "ble/application/Terminal.hpp"
#include "ble/hardware/HardwareAbstraction.hpp"
#include "infra/util/BoundedString.hpp"
#include "services/tracer/Tracer.hpp"

namespace application
{
    class TerminalInteractor
    {
    public:
        TerminalInteractor(application::TerminalCommands& terminal, services::Tracer& tracer, HardwareAbstraction::GapPeripheral& gapPeripheral);

    private:
        void AutoTune();
        void SetKpKiKd(const infra::BoundedConstString& param);
        void SetSpeed(const infra::BoundedConstString& param);
        void SetCurrent(const infra::BoundedConstString& param);
        void Start();
        void Stop();

    private:
        application::TerminalCommands& terminal;
        services::Tracer& tracer;
        application::GapPeripheralFlagsDecorator gapPeripheralDecorator;
    };
}

#endif
