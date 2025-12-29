#include "source/services/cli/FocTorqueInteractorImpl.hpp"

namespace services
{
    FocTorqueInteractorImpl::FocTorqueInteractorImpl(foc::Volts vdc, foc::TorqueController& foc)
        : FocInteractorImpl<foc::TorqueController>(vdc, foc)
    {}

    void FocTorqueInteractorImpl::SetTorque(const foc::Nm& torque)
    {
        foc::IdAndIqPoint setPoint{};

        setPoint.first = foc::Ampere{ torque.Value() };
        setPoint.second = foc::Ampere{ 0.0f };

        Foc().SetPoint(setPoint);
    }
}
