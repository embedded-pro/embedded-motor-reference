#include "application/hardware/PidAdapter.hpp"

namespace application
{
    PidDriverImpl::PidDriverImpl(application::HardwareFactory& hardware)
    {
    }

    void PidDriverImpl::Read(const infra::Function<void(float)>& onDone)
    {
    }

    void PidDriverImpl::ControlAction(float)
    {
    }

    void PidDriverImpl::Start(std::chrono::system_clock::duration sampleTime)
    {
    }

    void PidDriverImpl::Stop()
    {
    }

    std::chrono::system_clock::duration PidDriverImpl::SampleTime() const
    {
        return std::chrono::milliseconds(100);
    }
}
