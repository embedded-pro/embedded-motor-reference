#include "source/foc/implementations/test_doubles/DriversMock.hpp"
#include "source/services/parameter_identification/MotorIdentificationImpl.hpp"
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
        services::MotorIdentificationImpl identification{ driverMock, encoderMock, vdc };
    };
}

TEST_F(MotorIdentificationTest, GetResistance_ConfiguresCorrectPwmDutyCycles)
{
    services::MotorIdentificationImpl::ResistanceConfig config;
    config.testVoltagePercent = hal::Percent{ 5 };

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
    services::MotorIdentificationImpl::ResistanceConfig config;
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
    services::MotorIdentificationImpl::ResistanceConfig config;
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
    services::MotorIdentificationImpl::ResistanceConfig config;
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
    services::MotorIdentificationImpl::ResistanceConfig config;
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
    services::MotorIdentificationImpl::ResistanceConfig config;
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
    services::MotorIdentificationImpl::ResistanceConfig config;
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
    services::MotorIdentificationImpl::ResistanceConfig config;
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

TEST_F(MotorIdentificationTest, GetNumberOfPolePairs_StoresCallback)
{
    services::MotorIdentificationImpl::PolePairsConfig config;
    bool callbackCalled = false;

    EXPECT_CALL(encoderMock, Read()).Times(1);
    EXPECT_CALL(driverMock, Stop()).Times(1);
    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });

    identification.GetNumberOfPolePairs(config, [&callbackCalled](std::optional<uint8_t>)
        {
            callbackCalled = true;
        });

    EXPECT_FALSE(callbackCalled);
}

TEST_F(MotorIdentificationTest, GetResistance_WithDifferentVdc)
{
    foc::Volts customVdc{ 48.0f };
    services::MotorIdentificationImpl customIdentification{ driverMock, encoderMock, customVdc };

    services::MotorIdentificationImpl::ResistanceConfig config;
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

TEST_F(MotorIdentificationTest, GetInductance_ConfiguresCorrectPwmDutyCycles)
{
    services::MotorIdentificationImpl::InductanceConfig config;
    config.testVoltagePercent = hal::Percent{ 10 };
    config.resistance = foc::Ohm{ 1.0f };
    config.samplingFrequency = hal::Hertz{ 10000 };

    foc::PhasePwmDutyCycles expectedPwm{
        hal::Percent{ 55 },
        hal::Percent{ 45 },
        hal::Percent{ 50 }
    };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(PhasePwmDutyCyclesEq(expectedPwm))).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(hal::Hertz{ 10000 }, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });

    bool callbackCalled = false;
    identification.GetInductance(config, [&callbackCalled](std::optional<foc::Henry>)
        {
            callbackCalled = true;
        });

    EXPECT_FALSE(callbackCalled);
}

TEST_F(MotorIdentificationTest, GetInductance_CalculatesCorrectInductance)
{
    services::MotorIdentificationImpl::InductanceConfig config;
    config.testVoltagePercent = hal::Percent{ 10 };
    config.resistance = foc::Ohm{ 1.0f };
    config.samplingFrequency = hal::Hertz{ 10000 };
    config.minCurrentChange = foc::Ampere{ 0.1f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Henry> measuredInductance;
    identification.GetInductance(config, [&measuredInductance](std::optional<foc::Henry> inductance)
        {
            measuredInductance = inductance;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 2.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    ASSERT_TRUE(measuredInductance.has_value());
    EXPECT_NEAR(measuredInductance->Value(), 0.00009f, 0.00001f);
}

TEST_F(MotorIdentificationTest, GetInductance_ReturnsNulloptWhenCurrentChangeIsInsufficient)
{
    services::MotorIdentificationImpl::InductanceConfig config;
    config.testVoltagePercent = hal::Percent{ 10 };
    config.resistance = foc::Ohm{ 1.0f };
    config.samplingFrequency = hal::Hertz{ 10000 };
    config.minCurrentChange = foc::Ampere{ 1.0f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Henry> measuredInductance = foc::Henry{ 999.0f };
    identification.GetInductance(config, [&measuredInductance](std::optional<foc::Henry> inductance)
        {
            measuredInductance = inductance;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.01f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    EXPECT_FALSE(measuredInductance.has_value());
}

TEST_F(MotorIdentificationTest, GetInductance_WithHigherSamplingFrequency)
{
    services::MotorIdentificationImpl::InductanceConfig config;
    config.testVoltagePercent = hal::Percent{ 10 };
    config.resistance = foc::Ohm{ 0.5f };
    config.samplingFrequency = hal::Hertz{ 20000 };
    config.minCurrentChange = foc::Ampere{ 0.1f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(hal::Hertz{ 20000 }, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    std::optional<foc::Henry> measuredInductance;
    identification.GetInductance(config, [&measuredInductance](std::optional<foc::Henry> inductance)
        {
            measuredInductance = inductance;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 3.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    ASSERT_TRUE(measuredInductance.has_value());
    EXPECT_NEAR(measuredInductance->Value(), 0.000035f, 0.000001f);
}

TEST_F(MotorIdentificationTest, GetInductance_DoesNotCallbackAfterFirstSample)
{
    services::MotorIdentificationImpl::InductanceConfig config;
    config.resistance = foc::Ohm{ 1.0f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    int callbackCount = 0;
    identification.GetInductance(config, [&callbackCount](std::optional<foc::Henry>)
        {
            callbackCount++;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    EXPECT_EQ(callbackCount, 0);

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 2.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    EXPECT_EQ(callbackCount, 1);
}

TEST_F(MotorIdentificationTest, GetInductance_StopsDriverBeforeCallback)
{
    services::MotorIdentificationImpl::InductanceConfig config;
    config.resistance = foc::Ohm{ 1.0f };

    InSequence seq;
    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(1);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .WillOnce([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(1);

    bool callbackCalled = false;
    identification.GetInductance(config, [&callbackCalled](std::optional<foc::Henry>)
        {
            callbackCalled = true;
        });

    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });
    driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 2.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    EXPECT_TRUE(callbackCalled);
}

TEST_F(MotorIdentificationTest, GetNumberOfPolePairs_CalculatesCorrectPolePairs)
{
    services::MotorIdentificationImpl::PolePairsConfig config;
    config.testVoltagePercent = hal::Percent{ 20 };
    config.electricalRevolutions = 1;
    config.minMechanicalRotation = foc::Radians{ 0.1f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(12);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(hal::Hertz{ 100 }, _))
        .Times(12)
        .WillRepeatedly([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(encoderMock, Read())
        .Times(2)
        .WillOnce(::testing::Return(foc::Radians{ 0.0f }))
        .WillOnce(::testing::Return(foc::Radians{ 3.14159265f }));
    EXPECT_CALL(driverMock, Stop()).Times(13);

    std::optional<std::size_t> measuredPolePairs;
    identification.GetNumberOfPolePairs(config, [&measuredPolePairs](std::optional<std::size_t> polePairs)
        {
            measuredPolePairs = polePairs;
        });

    for (size_t i = 0; i < 12; ++i)
        driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    ASSERT_TRUE(measuredPolePairs.has_value());
    EXPECT_EQ(*measuredPolePairs, 2);
}

TEST_F(MotorIdentificationTest, GetNumberOfPolePairs_ReturnsNulloptWhenRotationInsufficient)
{
    services::MotorIdentificationImpl::PolePairsConfig config;
    config.testVoltagePercent = hal::Percent{ 20 };
    config.electricalRevolutions = 1;
    config.minMechanicalRotation = foc::Radians{ 1.0f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(12);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .Times(12)
        .WillRepeatedly([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(encoderMock, Read())
        .Times(2)
        .WillOnce(::testing::Return(foc::Radians{ 0.0f }))
        .WillOnce(::testing::Return(foc::Radians{ 0.01f }));
    EXPECT_CALL(driverMock, Stop()).Times(13);

    std::optional<std::size_t> measuredPolePairs = std::make_optional<std::size_t>(999);
    identification.GetNumberOfPolePairs(config, [&measuredPolePairs](std::optional<std::size_t> polePairs)
        {
            measuredPolePairs = polePairs;
        });

    for (size_t i = 0; i < 12; ++i)
        driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    EXPECT_FALSE(measuredPolePairs.has_value());
}

TEST_F(MotorIdentificationTest, GetNumberOfPolePairs_WithMoreElectricalSteps)
{
    services::MotorIdentificationImpl::PolePairsConfig config;
    config.testVoltagePercent = hal::Percent{ 20 };
    config.electricalRevolutions = 2;
    config.minMechanicalRotation = foc::Radians{ 0.1f };

    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(24);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .Times(24)
        .WillRepeatedly([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });

    EXPECT_CALL(encoderMock, Read())
        .Times(2)
        .WillOnce(::testing::Return(foc::Radians{ 0.0f }))
        .WillOnce(::testing::Return(foc::Radians{ 6.28318531f }));

    EXPECT_CALL(driverMock, Stop()).Times(25);

    std::optional<std::size_t> measuredPolePairs;
    identification.GetNumberOfPolePairs(config, [&measuredPolePairs](std::optional<std::size_t> polePairs)
        {
            measuredPolePairs = polePairs;
        });

    for (size_t i = 0; i < 24; ++i)
        driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    ASSERT_TRUE(measuredPolePairs.has_value());
    EXPECT_EQ(*measuredPolePairs, 2);
}

TEST_F(MotorIdentificationTest, GetNumberOfPolePairs_StopsDriverAfterCompletion)
{
    services::MotorIdentificationImpl::PolePairsConfig config;
    config.electricalRevolutions = 1;

    EXPECT_CALL(encoderMock, Read())
        .Times(2)
        .WillOnce(::testing::Return(foc::Radians{ 0.0f }))
        .WillOnce(::testing::Return(foc::Radians{ 0.5f }));
    EXPECT_CALL(driverMock, ThreePhasePwmOutput(_)).Times(12);
    EXPECT_CALL(driverMock, PhaseCurrentsReady(_, _))
        .Times(12)
        .WillRepeatedly([this](auto, const auto& onDone)
            {
                driverMock.StorePhaseCurrentsCallback(onDone);
            });
    EXPECT_CALL(driverMock, Stop()).Times(13);

    bool callbackCalled = false;
    identification.GetNumberOfPolePairs(config, [&callbackCalled](std::optional<std::size_t>)
        {
            callbackCalled = true;
        });

    for (size_t i = 0; i < 12; ++i)
        driverMock.TriggerPhaseCurrentsCallback({ foc::Ampere{ 1.0f }, foc::Ampere{ 0.0f }, foc::Ampere{ 0.0f } });

    EXPECT_TRUE(callbackCalled);
}
