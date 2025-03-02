#include "application/hardware/HardwareFactory.hpp"
#include "application/motors/DC/instantiations/Logic.hpp"

int main()
{
    static std::optional<application::Logic> logic;
    // static application::HardwareImpl hardware([]()
    //     {
    //         logic.emplace(hardware);
    //     });

    // hardware.Run();
    __builtin_unreachable();
}
