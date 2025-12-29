#pragma once

#include "source/foc/interfaces/FieldOrientedController.hpp"
#include "source/services/cli/FocInteractor.hpp"

namespace services
{
    template<typename FocImpl>
    class FocInteractorImpl
        : public FocInteractor
    {
    public:
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters) override;
        void Start() override;
        void Stop() override;

        foc::Volts Vdc();
        foc::IdAndIqTunings& CurrentTunings();
        FocImpl& Foc();

    protected:
        explicit FocInteractorImpl(foc::Volts vdc, FocImpl& focImpl);

    private:
        foc::Volts vdc;
        FocImpl& focImpl;
        foc::IdAndIqTunings tunings{};
    };

    // Implementation

    template<typename FocImpl>
    FocInteractorImpl<FocImpl>::FocInteractorImpl(foc::Volts vdc, FocImpl& focImpl)
        : vdc(vdc)
        , focImpl(focImpl)
    {}

    template<typename FocImpl>
    void FocInteractorImpl<FocImpl>::AutoTune(const infra::Function<void()>& onDone)
    {
        onDone();
    };

    template<typename FocImpl>
    void FocInteractorImpl<FocImpl>::SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters)
    {
        const auto& dParams = pidDAndQParameters.first;
        if (dParams.kp.has_value())
            tunings.first.kp = *dParams.kp;
        if (dParams.ki.has_value())
            tunings.first.ki = *dParams.ki;
        if (dParams.kd.has_value())
            tunings.first.kd = *dParams.kd;

        const auto& qParams = pidDAndQParameters.second;
        if (qParams.kp.has_value())
            tunings.second.kp = *qParams.kp;
        if (qParams.ki.has_value())
            tunings.second.ki = *qParams.ki;
        if (qParams.kd.has_value())
            tunings.second.kd = *qParams.kd;

        focImpl.SetDQPidParameters(tunings);
    };

    template<typename FocImpl>
    void FocInteractorImpl<FocImpl>::Start()
    {
        focImpl.Enable();
    };

    template<typename FocImpl>
    void FocInteractorImpl<FocImpl>::Stop()
    {
        focImpl.Disable();
    };

    template<typename FocImpl>
    foc::Volts FocInteractorImpl<FocImpl>::Vdc()
    {
        return vdc;
    };

    template<typename FocImpl>
    foc::IdAndIqTunings& FocInteractorImpl<FocImpl>::CurrentTunings()
    {
        return tunings;
    };

    template<typename FocImpl>
    FocImpl& FocInteractorImpl<FocImpl>::Foc()
    {
        return focImpl;
    };
}
