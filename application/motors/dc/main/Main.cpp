#include HARDWARE_FACTORY_IMPL_HEADER
#include "application/motors/dc/instantiations/Logic.hpp"

int main()
{
    static std::optional<application::Logic> logic;
    static application::HardwareFactoryImpl hardware([]()
        {
            logic.emplace(hardware);
        });

    hardware.Run();
    
#if defined(__GNUC__) || defined(__clang__)
    __builtin_unreachable();
#elif defined(_MSC_VER)
    __assume(false);
#endif
}
