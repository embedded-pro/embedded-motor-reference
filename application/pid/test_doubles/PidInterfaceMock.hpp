#pragma once

#include "application/pid/PidInterface.hpp"
#include <gmock/gmock.h>

namespace application
{
    class PidInterfaceMock
        : public PidInterface
    {
    public:
        void Read(const infra::Function<void(float)>& onDone) override
        {
            MockRead(onDone);
            savedCallback = onDone;
        }

        void TriggerCallback(int32_t value)
        {
            if (savedCallback)
                savedCallback(value);
        }

        MOCK_METHOD(void, MockRead, (const infra::Function<void(float)>&));
        MOCK_METHOD(void, ControlAction, (float), (override));
        MOCK_METHOD(void, Start, (infra::Duration sampleTime), (override));
        MOCK_METHOD(void, Stop, (), (override));

    private:
        infra::Function<void(float)> savedCallback;
    };
}
