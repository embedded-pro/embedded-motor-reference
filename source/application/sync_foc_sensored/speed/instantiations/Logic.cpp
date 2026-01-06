#include "source/application/sync_foc_sensored/speed/instantiations/Logic.hpp"

namespace
{
    std::chrono::system_clock::duration TimeStepFromFrequency(hal::Hertz frequency)
    {
        return std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::nanoseconds(static_cast<int64_t>(1'000'000'000LL / frequency.Value())));
    }
}

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : hardwareAdapter{ hardware }
        , debugLed{ hardware.Leds().front(), std::chrono::milliseconds(50), std::chrono::milliseconds(1950) }
        , vdc{ hardware.PowerSupplyVoltage() }
        , terminalWithStorage{ hardware.Terminal(), hardware.Tracer(), services::TerminalWithBanner::Banner{ "sync_foc_sensored:speed", hardware.PowerSupplyVoltage(), hardware.SystemClock() } }
        , motorStateMachine(
              TerminalAndTracer{ terminalWithStorage, hardware.Tracer() },
              MotorDriverAndEncoder{ hardwareAdapter, hardwareAdapter },
              vdc, hardware.MaxCurrentSupported(), TimeStepFromFrequency(hardwareAdapter.BaseFrequency()))
    {}
}
