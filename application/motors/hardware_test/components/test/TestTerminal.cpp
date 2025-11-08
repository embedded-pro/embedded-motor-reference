#include "application/motors/hardware_test/components/Terminal.hpp"
#include "foc/interfaces/Driver.hpp"
#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/event/test_helper/EventDispatcherWithWeakPtrFixture.hpp"
#include "infra/util/test_helper/MockHelpers.hpp"
#include "infra/util/test_helper/ProxyCreatorMock.hpp"
#include "services/tracer/Tracer.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace
{
    class StreamWriterMock
        : public infra::StreamWriter
    {
    public:
        MOCK_METHOD(void, Insert, (infra::ConstByteRange range, infra::StreamErrorPolicy& errorPolicy), (override));
        MOCK_METHOD(std::size_t, Available, (), (const override));
        MOCK_METHOD(std::size_t, ConstructSaveMarker, (), (const override));
        MOCK_METHOD(std::size_t, GetProcessedBytesSince, (std::size_t marker), (const override));
        MOCK_METHOD(infra::ByteRange, SaveState, (std::size_t marker), (override));
        MOCK_METHOD(void, RestoreState, (infra::ByteRange range), (override));
        MOCK_METHOD(infra::ByteRange, Overwrite, (std::size_t marker), (override));
    };

    class HardwareFactoryMock
        : public application::HardwareFactory
    {
    public:
        MOCK_METHOD(void, Run, (), (override));
        MOCK_METHOD(services::Tracer&, Tracer, (), (override));
        MOCK_METHOD(services::TerminalWithCommands&, Terminal, (), (override));
        MOCK_METHOD(infra::MemoryRange<hal::GpioPin>, Leds, (), (override));
        MOCK_METHOD(hal::PerformanceTracker&, PerformanceTimer, (), (override));
        MOCK_METHOD(hal::Hertz, BaseFrequency, (), (const, override));
        MOCK_METHOD(foc::Volts, PowerSupplyVoltage, (), (override));
        MOCK_METHOD(foc::Ampere, MaxCurrentSupported, (), (override));
        MOCK_METHOD((infra::CreatorBase<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds, hal::Hertz)>&), SynchronousThreeChannelsPwmCreator, (), (override));
        MOCK_METHOD((infra::CreatorBase<hal::AdcMultiChannel, void(SampleAndHold)>&), AdcMultiChannelCreator, (), (override));
        MOCK_METHOD((infra::CreatorBase<hal::SynchronousQuadratureEncoder, void()>&), SynchronousQuadratureEncoderCreator, (), (override));
    };

    class PwmMock
        : public hal::SynchronousThreeChannelsPwm
    {
    public:
        MOCK_METHOD(void, SetBaseFrequency, (hal::Hertz baseFrequency), (override));
        MOCK_METHOD(void, Stop, (), (override));
        MOCK_METHOD(void, Start, (hal::Percent dutyCycle1, hal::Percent dutyCycle2, hal::Percent dutyCycle3), (override));
    };

    class AdcMock
        : public hal::AdcMultiChannel
    {
    public:
        MOCK_METHOD(void, Measure, (const infra::Function<void(Samples)>& onDone), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };

    class EncoderMock
        : public hal::SynchronousQuadratureEncoder
    {
    public:
        MOCK_METHOD(uint32_t, Position, (), (override));
        MOCK_METHOD(uint32_t, Resolution, (), (override));
        MOCK_METHOD(MotionDirection, Direction, (), (override));
        MOCK_METHOD(uint32_t, Speed, (), (override));
    };

    class PerformanceTrackerMock
        : public hal::PerformanceTracker
    {
    public:
        MOCK_METHOD(void, Start, (), (override));
        MOCK_METHOD(uint32_t, ElapsedCycles, (), (override));
    };

    class TestHardwareTerminal
        : public testing::Test
        , public infra::EventDispatcherWithWeakPtrFixture
    {
    public:
        TestHardwareTerminal()
        {
            EXPECT_CALL(hardwareFactoryMock, Terminal()).WillRepeatedly(testing::ReturnRef(terminalWithCommands));
            EXPECT_CALL(hardwareFactoryMock, Tracer()).WillRepeatedly(testing::ReturnRef(tracer));
            EXPECT_CALL(hardwareFactoryMock, SynchronousThreeChannelsPwmCreator()).WillRepeatedly(testing::ReturnRef(pwmCreator));
            EXPECT_CALL(hardwareFactoryMock, AdcMultiChannelCreator()).WillRepeatedly(testing::ReturnRef(adcCreator));
            EXPECT_CALL(hardwareFactoryMock, SynchronousQuadratureEncoderCreator()).WillRepeatedly(testing::ReturnRef(encoderCreator));
            EXPECT_CALL(hardwareFactoryMock, PerformanceTimer()).WillRepeatedly(testing::ReturnRef(performanceTrackerMock));
            EXPECT_CALL(hardwareFactoryMock, PowerSupplyVoltage()).WillRepeatedly(testing::Return(foc::Volts{ 24.0f }));
            EXPECT_CALL(hardwareFactoryMock, MaxCurrentSupported()).WillRepeatedly(testing::Return(foc::Ampere{ 5.0f }));
            EXPECT_CALL(hardwareFactoryMock, BaseFrequency()).WillRepeatedly(testing::Return(hal::Hertz{ 10000 }));

            EXPECT_CALL(encoderCreator, Constructed());
            EXPECT_CALL(pwmCreator, Constructed(std::chrono::nanoseconds{ 500 }, hal::Hertz{ 10000 }));
            EXPECT_CALL(adcCreator, Constructed(application::HardwareFactory::SampleAndHold::medium));

            EXPECT_CALL(adcMock, Measure(testing::_)).WillRepeatedly(testing::SaveArg<0>(&onAdcMeasurementDone));

            terminalInteractor.emplace(terminal, hardwareFactoryMock);
        }

        ~TestHardwareTerminal()
        {
            EXPECT_CALL(encoderCreator, Destructed()).Times(1);
            EXPECT_CALL(pwmCreator, Destructed()).Times(testing::AtLeast(1));
            EXPECT_CALL(adcCreator, Destructed()).Times(testing::AtLeast(1));
        }

        testing::StrictMock<HardwareFactoryMock> hardwareFactoryMock;
        testing::StrictMock<StreamWriterMock> streamWriterMock;
        infra::TextOutputStream::WithErrorPolicy stream{ streamWriterMock };
        services::TracerToStream tracer{ stream };
        testing::StrictMock<hal::SerialCommunicationMock> communication;
        infra::Execute execute{ [this]()
            {
                EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
            } };
        services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<128, 5> terminalWithCommands{ communication, tracer };
        services::TerminalWithStorage::WithMaxSize<10> terminal{ terminalWithCommands, tracer };

        testing::StrictMock<PwmMock> pwmMock;
        testing::StrictMock<AdcMock> adcMock;
        testing::StrictMock<EncoderMock> encoderMock;
        testing::NiceMock<PerformanceTrackerMock> performanceTrackerMock;

        infra::CreatorMock<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds, hal::Hertz)> pwmCreator{ pwmMock };
        infra::CreatorMock<hal::AdcMultiChannel, void(application::HardwareFactory::SampleAndHold)> adcCreator{ adcMock };
        infra::CreatorMock<hal::SynchronousQuadratureEncoder, void()> encoderCreator{ encoderMock };

        std::optional<application::TerminalInteractor> terminalInteractor;
        infra::Function<void(hal::AdcMultiChannel::Samples)> onAdcMeasurementDone;

        void InvokeCommand(std::string command, const std::function<void()>& onCommandReceived)
        {
            ::testing::InSequence _;

            for (const auto& data : command)
                EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ static_cast<uint8_t>(data) }), testing::_));

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            onCommandReceived();
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '>', ' ' } }), testing::_));

            communication.dataReceived(infra::MakeStringByteRange(command + "\r"));
        }

        void Payload(std::string header, std::string mean, std::string deviationStandard, std::string footer)
        {
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));

            for (char c : mean)
                EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ static_cast<uint8_t>(c) }), testing::_));

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ ',' }), testing::_));

            for (char c : deviationStandard)
                EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ static_cast<uint8_t>(c) }), testing::_));

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(footer.begin(), footer.end())), testing::_));
        }
    };
}

TEST_F(TestHardwareTerminal, stop_command)
{
    InvokeCommand("stop", [this]()
        {
            EXPECT_CALL(pwmMock, Stop());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, stop_alias)
{
    InvokeCommand("stp", [this]()
        {
            EXPECT_CALL(pwmMock, Stop());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, read_command_with_no_data)
{
    InvokeCommand("read", [this]()
        {
            ::testing::InSequence _;

            std::string headerMessage = "  Measures [average,standard deviation]";
            std::string expectedMessage = "    No ADC data available";

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(headerMessage.begin(), headerMessage.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(expectedMessage.begin(), expectedMessage.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, read_command_with_adc_data)
{
    std::array<uint16_t, 5> sampleData = { 1000, 1500, 2000, 2500, 3000 };
    hal::AdcMultiChannel::Samples simulatedSamples{ sampleData };
    onAdcMeasurementDone(simulatedSamples);
    onAdcMeasurementDone(simulatedSamples);
    onAdcMeasurementDone(simulatedSamples);

    InvokeCommand("read", [this]()
        {
            ::testing::InSequence _;

            std::string headerMessage = "  Measures [average,standard deviation]";

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(headerMessage.begin(), headerMessage.end())), testing::_));
            Payload("    Phase A:            [", "1000", "0", "]");
            Payload("    Phase B:            [", "1500", "0", "]");
            Payload("    Phase C:            [", "2000", "0", "]");
            Payload("    Reference Voltage:  [", "2500", "0", "]");
            Payload("    Total Current:      [", "3000", "0", "]");
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, read_alias_with_no_data)
{
    InvokeCommand("r", [this]()
        {
            ::testing::InSequence _;

            std::string headerMessage = "  Measures [average,standard deviation]";
            std::string expectedMessage = "    No ADC data available";

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(headerMessage.begin(), headerMessage.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(expectedMessage.begin(), expectedMessage.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, read_alias_with_multiple_samples)
{
    std::array<uint16_t, 5> sampleData1 = { 280, 780, 1280, 1780, 2280 };
    std::array<uint16_t, 5> sampleData2 = { 920, 1420, 1920, 2420, 2920 };
    std::array<uint16_t, 5> sampleData3 = { 1080, 1580, 2080, 2580, 3080 };
    std::array<uint16_t, 5> sampleData4 = { 1720, 2220, 2720, 3220, 3720 };

    onAdcMeasurementDone(hal::AdcMultiChannel::Samples{ sampleData1 });
    onAdcMeasurementDone(hal::AdcMultiChannel::Samples{ sampleData2 });
    onAdcMeasurementDone(hal::AdcMultiChannel::Samples{ sampleData3 });
    onAdcMeasurementDone(hal::AdcMultiChannel::Samples{ sampleData4 });

    InvokeCommand("r", [this]()
        {
            ::testing::InSequence _;

            std::string headerMessage = "  Measures [average,standard deviation]";

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(headerMessage.begin(), headerMessage.end())), testing::_));
            Payload("    Phase A:            [", "1000", "512", "]");
            Payload("    Phase B:            [", "1500", "512", "]");
            Payload("    Phase C:            [", "2000", "512", "]");
            Payload("    Reference Voltage:  [", "2500", "512", "]");
            Payload("    Total Current:      [", "3000", "512", "]");
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, duty_command)
{
    InvokeCommand("duty 10 20 30", [this]()
        {
            EXPECT_CALL(pwmMock, Start(hal::Percent{ 10 }, hal::Percent{ 20 }, hal::Percent{ 30 }));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, duty_alias)
{
    InvokeCommand("d 15 25 35", [this]()
        {
            EXPECT_CALL(pwmMock, Start(hal::Percent{ 15 }, hal::Percent{ 25 }, hal::Percent{ 35 }));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, duty_invalid_argument_count)
{
    InvokeCommand("duty 10 20", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid number of arguments" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, pwm_command)
{
    InvokeCommand("pwm 500 10000", [this]()
        {
            EXPECT_CALL(pwmCreator, Destructed()).Times(1);
            EXPECT_CALL(pwmCreator, Constructed(std::chrono::nanoseconds{ 500 }, hal::Hertz{ 10000 })).Times(1);
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, pwm_alias)
{
    InvokeCommand("p 750 15000", [this]()
        {
            EXPECT_CALL(pwmCreator, Destructed()).Times(1);
            EXPECT_CALL(pwmCreator, Constructed(std::chrono::nanoseconds{ 750 }, hal::Hertz{ 15000 })).Times(1);
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, adc_command)
{
    InvokeCommand("adc medium", [this]()
        {
            EXPECT_CALL(adcCreator, Constructed(application::HardwareFactory::SampleAndHold::medium)).Times(1);
            EXPECT_CALL(adcMock, Measure(testing::_)).Times(1);
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, adc_alias)
{
    InvokeCommand("a shortest", [this]()
        {
            EXPECT_CALL(adcCreator, Constructed(application::HardwareFactory::SampleAndHold::shortest)).Times(1);
            EXPECT_CALL(adcMock, Measure(testing::_)).Times(1);
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, adc_invalid_value)
{
    InvokeCommand("adc invalid", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value. It should be one of: shortest, shorter, medium, longer, longest." };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, pid_command)
{
    InvokeCommand("pid 1.0 0.5 0.1 2.0 1.0 0.2", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, pid_alias)
{
    InvokeCommand("c 1.5 0.75 0.15 2.5 1.5 0.25", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, pid_invalid_argument_count)
{
    InvokeCommand("pid 1.0 0.5 0.1", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid number of arguments" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, pid_invalid_speed_kp)
{
    InvokeCommand("pid invalid 0.5 0.1 2.0 1.0 0.2", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value for Speed Kp" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, pid_invalid_dq_ki)
{
    InvokeCommand("pid 1.0 0.5 0.1 2.0 invalid 0.2", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value for DQ-axis Ki" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_command)
{
    InvokeCommand("foc 45.0 1.5 2.0 2.5", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1000));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_alias)
{
    InvokeCommand("f 90.0 2.0 3.0 4.0", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1500));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_invalid_argument_count)
{
    InvokeCommand("foc 45.0 1.5", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid number of arguments" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_invalid_angle)
{
    InvokeCommand("foc 400.0 1.5 2.0 2.5", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value for angle. It should be a float between -360 and 360." };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_invalid_phase_a_current)
{
    InvokeCommand("foc 45.0 invalid 2.0 2.5", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value for phase A current. It should be a float between -1000 and 1000." };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_invalid_phase_c_current)
{
    InvokeCommand("foc 45.0 1.5 2.0 1500.0", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value for phase C current. It should be a float between -1000 and 1000." };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_with_negative_angle)
{
    InvokeCommand("foc -90.0 1.5 2.0 2.5", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1200));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_with_negative_currents)
{
    InvokeCommand("foc 45.0 -1.5 -2.0 -2.5", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1100));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_simulation_output_format)
{
    InvokeCommand("foc 0.0 0.0 0.0 0.0", [this]()
        {
            ::testing::InSequence _;

            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(2000));

            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_with_maximum_angle)
{
    InvokeCommand("foc 360.0 1.0 2.0 3.0", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1300));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_with_minimum_angle)
{
    InvokeCommand("foc -360.0 1.0 2.0 3.0", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1250));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_with_maximum_currents)
{
    InvokeCommand("foc 45.0 1000.0 1000.0 1000.0", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1400));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_with_minimum_currents)
{
    InvokeCommand("foc 45.0 -1000.0 -1000.0 -1000.0", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1350));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_multiple_simulations)
{
    InvokeCommand("foc 30.0 1.0 2.0 3.0", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1050));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();

    InvokeCommand("foc 60.0 2.0 3.0 4.0", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1150));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, foc_with_fractional_values)
{
    InvokeCommand("foc 45.5 1.234 2.567 3.891", [this]()
        {
            EXPECT_CALL(performanceTrackerMock, Start());
            EXPECT_CALL(performanceTrackerMock, ElapsedCycles()).WillOnce(testing::Return(1075));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, motor_command)
{
    InvokeCommand("motor 14", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, motor_alias)
{
    InvokeCommand("m 8", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, motor_invalid_argument_count)
{
    InvokeCommand("motor 14 8", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid number of arguments" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, motor_invalid_poles_too_low)
{
    InvokeCommand("motor 1", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value for poles. It should be an integer between 2 and 16." };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, motor_invalid_poles_too_high)
{
    InvokeCommand("motor 18", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value for poles. It should be an integer between 2 and 16." };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, motor_invalid_poles_not_a_number)
{
    InvokeCommand("motor invalid", [this]()
        {
            ::testing::InSequence _;

            std::string newline{ "\r\n" };
            std::string header{ "ERROR: " };
            std::string payload{ "invalid value for poles. It should be an integer between 2 and 16." };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, motor_minimum_valid_poles)
{
    InvokeCommand("motor 2", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, motor_maximum_valid_poles)
{
    InvokeCommand("motor 16", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AnyNumber());
        });

    ExecuteAllActions();
}
