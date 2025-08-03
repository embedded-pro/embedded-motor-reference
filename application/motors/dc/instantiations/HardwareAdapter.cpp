#include "application/motors/dc/instantiations/HardwareAdapter.hpp"

namespace application
{
    HardwareAdapter::HardwareAdapter(application::HardwareFactory& hardware)
    {
    }

    void HardwareAdapter::Read(const infra::Function<void(float)>& onDone)
    {
    }

    void HardwareAdapter::ControlAction(float)
    {
    }

    void HardwareAdapter::Start(infra::Duration sampleTime)
    {
    }

    void HardwareAdapter::Stop()
    {
    }
}
