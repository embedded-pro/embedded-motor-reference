#pragma once

#include "application/pid/PidInterface.hpp"
#include <gmock/gmock.h>

namespace application
{
    class PidInterfaceMock
        : public PidInterface
    {
    public:
        MOCK_METHOD(void, Read, (const infra::Function<void(float)>&), (override));
        MOCK_METHOD(void, ControlAction, (float), (override));
        MOCK_METHOD(void, Start, (infra::Duration sampleTime), (override));
        MOCK_METHOD(void, Stop, (), (override));

        void StoreReadCallback(const infra::Function<void(float)>& onDone)
        {
            savedCallback = onDone;
        }

        void TriggerCallback(int32_t value)
        {
            if (savedCallback)
                savedCallback(value);
        }

    private:
        infra::Function<void(float)> savedCallback;
    };
}
