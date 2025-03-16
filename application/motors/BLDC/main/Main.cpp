#include HARDWARE_FACTORY_IMPL_HEADER
#include "application/motors/BLDC/instantiations/Logic.hpp"
#include <optional>

int main()
{
    static std::optional<application::Logic> logic;
    static application::HardwareFactoryImpl hardware([]()
        {
            logic.emplace(hardware);
        });

    hardware.Run();
    __builtin_unreachable();
}
