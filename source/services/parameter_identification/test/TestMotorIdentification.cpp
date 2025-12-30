#include "source/foc/implementations/test_doubles/DriversMock.hpp"
#include "source/services/parameter_identification/MotorIdentification.hpp"
#include <gmock/gmock.h>

namespace
{
    using namespace testing;

    MATCHER_P(PhasePwmDutyCyclesEq, expected, "")
    {
        return arg.a.Value() == expected.a.Value() &&
               arg.b.Value() == expected.b.Value() &&
               arg.c.Value() == expected.c.Value();
    }

    class MotorIdentificationTest
        : public ::testing::Test
    {
    public:
        StrictMock<foc::FieldOrientedControllerInterfaceMock> driverMock;
        StrictMock<foc::EncoderMock> encoderMock;
        foc::Volts vdc{ 24.0f };
        services::MotorIdentificationWithAlignment identification{ driverMock, encoderMock, vdc };
    };
}

TEST_F(MotorIdentificationTest, GetResistance_ConfiguresCorrectPwmDutyCycles)
{
    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.testVoltagePercent = hal::Percent{ 5 };

    // Expected duty cycles: 50 +/- (5 * 50 / 100) = 50 +/- 2 = 52, 48, 50
    foc::PhasePwmDutyCycles expectedPwm{
        hal::Percent{ 52 },
        hal::Percent{ 48 },
        hal::Percent{ 50 }
    };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(PhasePwmDutyCyclesEq(expectedPwm))).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(hal::Hertz{ 1000 }, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });

    bool callbackCalled = false;
    identification.GetResistance(config, [&callbackCalled](std::optional<foc::Ohm> resistance)
        {
            callbackCalled = true;
        });

    EXPECT_FALSE(callbackCalled);
}

TEST_F(MotorIdentificationTest, GetResistance_CalculatesCorrectResistanceAfterSampling)
{
    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.testVoltagePercent = hal::Percent{ 5 };
    config.sampleCount = 4;
    config.minCurrent = foc::Ampere{ 0.1f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Ohm> measuredResistance;
    identification.GetResistance(config, [&measuredResistance](std::optional<foc::Ohm> resistance)
        {
            measuredResistance = resistance;
        });

    foc::PhaseCurrents samples[] = {
        { foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } },
        { foc::Ampere{ 2.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } },
        { foc::Ampere{ 3.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } },
        { foc::Ampere{ 4.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } }
    };

    for (const auto& sample : samples)
    {
        driverMock.TriggerPhaseCurrentsCallback(sample);
    }

    ASSERT_TRUE(measuredResistance.has_value());
    EXPECT_NEAR(measuredResistance->Value(), 0.48f, 0.01f);
}

TEST_F(MotorIdentificationTest, GetResistance_Returns_Zero_When_Current_Below_Minimum)
{
    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.testVoltagePercent = hal::Percent{ 5 };
    config.sampleCount = 2;
    config.minCurrent = foc::Ampere{ 0.5f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Ohm> measuredResistance = foc::Ohm{ 999.0f };
    identification.GetResistance(config, [&measuredResistance](std::optional<foc::Ohm> resistance)
        {
            measuredResistance = resistance;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 0.01f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 0.02f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    EXPECT_FALSE(measuredResistance.has_value());
}

TEST_F(MotorIdentificationTest, GetResistance_Handles_Negative_Current)
{
    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.testVoltagePercent = hal::Percent{ 5 };
    config.sampleCount = 2;
    config.minCurrent = foc::Ampere{ 0.1f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Ohm> measuredResistance;
    identification.GetResistance(config, [&measuredResistance](std::optional<foc::Ohm> resistance)
        {
            measuredResistance = resistance;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ -1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ -2.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    ASSERT_TRUE(measuredResistance.has_value());
    EXPECT_NEAR(measuredResistance->Value(), -0.8f, 0.01f);
}

TEST_F(MotorIdentificationTest, GetResistance_WithCustomVoltagePercent)
{
    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.testVoltagePercent = hal::Percent{ 10 };
    config.sampleCount = 1;
    config.minCurrent = foc::Ampere{ 0.1f };

    foc::PhasePwmDutyCycles expectedPwm{
        hal::Percent{ 55 },
        hal::Percent{ 45 },
        hal::Percent{ 50 }
    };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(PhasePwmDutyCyclesEq(expectedPwm))).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Ohm> measuredResistance;
    identification.GetResistance(config, [&measuredResistance](std::optional<foc::Ohm> resistance)
        {
            measuredResistance = resistance;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 2.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    ASSERT_TRUE(measuredResistance.has_value());
    EXPECT_NEAR(measuredResistance->Value(), 1.2f, 0.01f);
}

TEST_F(MotorIdentificationTest, GetResistance_WithManySamples)
{
    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.testVoltagePercent = hal::Percent{ 5 };
    config.sampleCount = 16;
    config.minCurrent = foc::Ampere{ 0.1f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Ohm> measuredResistance;
    identification.GetResistance(config, [&measuredResistance](std::optional<foc::Ohm> resistance)
        {
            measuredResistance = resistance;
        });

    for (size_t i = 0; i < 16; ++i)
        driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 2.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    ASSERT_TRUE(measuredResistance.has_value());
    EXPECT_NEAR(measuredResistance->Value(), 0.6f, 0.01f);
}

TEST_F(MotorIdentificationTest, GetResistance_DoesNotCallbackBeforeAllSamplesCollected)
{
    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.sampleCount = 4;

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    int callbackCount = 0;
    identification.GetResistance(config, [&callbackCount](std::optional<foc::Ohm>)
        {
            callbackCount++;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    EXPECT_EQ(callbackCount, 0);

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    EXPECT_EQ(callbackCount, 0);

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    EXPECT_EQ(callbackCount, 0);

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    EXPECT_EQ(callbackCount, 1);
}

TEST_F(MotorIdentificationTest, GetResistance_StopsDriverBeforeCallback)
{
    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.sampleCount = 1;

    InSequence seq;
    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    bool callbackCalled = false;
    identification.GetResistance(config, [&callbackCalled](std::optional<foc::Ohm>)
        {
            callbackCalled = true;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    EXPECT_TRUE(callbackCalled);
}

TEST_F(MotorIdentificationTest, GetInductance_StoresCallback)
{
    bool callbackCalled = false;
    identification.GetInductance([&callbackCalled](std::optional<foc::Henry>)
        {
            callbackCalled = true;
        });

    EXPECT_FALSE(callbackCalled);
}

TEST_F(MotorIdentificationTest, GetNumberOfPolePairs_StoresCallback)
{
    bool callbackCalled = false;
    identification.GetNumberOfPolePairs([&callbackCalled](std::optional<std::size_t>)
        {
            callbackCalled = true;
        });

    EXPECT_FALSE(callbackCalled);
}

TEST_F(MotorIdentificationTest, AlignMotor_StoresCallback)
{
    bool callbackCalled = false;
    identification.AlignMotor(7, [&callbackCalled](std::optional<foc::Radians>)
        {
            callbackCalled = true;
        });

    EXPECT_FALSE(callbackCalled);
}

TEST_F(MotorIdentificationTest, GetResistance_WithDifferentVdc)
{
    foc::Volts customVdc{ 48.0f };
    services::MotorIdentificationWithAlignment customIdentification{ driverMock, encoderMock, customVdc };

    services::MotorIdentificationWithAlignment::ResistanceConfig config;
    config.testVoltagePercent = hal::Percent{ 5 };
    config.sampleCount = 1;
    config.minCurrent = foc::Ampere{ 0.1f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Ohm> measuredResistance;
    customIdentification.GetResistance(config, [&measuredResistance](std::optional<foc::Ohm> resistance)
        {
            measuredResistance = resistance;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 2.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    ASSERT_TRUE(measuredResistance.has_value());
    EXPECT_NEAR(measuredResistance->Value(), 1.2f, 0.01f);
}
