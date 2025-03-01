#include "application/dc/instantiations/Logic.hpp"
#include "application/hardware/HardwareFactory.hpp"

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
