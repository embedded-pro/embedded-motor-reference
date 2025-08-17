#include "application/motors/hardware_test/components/Terminal.hpp"
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
                EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '>', ' ' } }), testing::_));
            } };
        services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<128, 5> terminalWithCommands{ communication, tracer };
        services::TerminalWithStorage::WithMaxSize<10> terminal{ terminalWithCommands, tracer };

        testing::StrictMock<PwmMock> pwmMock;
        testing::StrictMock<AdcMock> adcMock;
        testing::StrictMock<EncoderMock> encoderMock;

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
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(2));

            std::string expectedMessage = "\t\tNo ADC data available";
            EXPECT_CALL(streamWriterMock, Insert(
                                              infra::CheckByteRangeContents(std::vector<uint8_t>(expectedMessage.begin(), expectedMessage.end())),
                                              testing::_))
                .Times(1);
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, read_command_with_adc_data)
{
    std::array<uint16_t, 5> sampleData = { 1000, 1500, 2000, 2500, 3000 };
    hal::AdcMultiChannel::Samples simulatedSamples{ sampleData };
    onAdcMeasurementDone(simulatedSamples);

    InvokeCommand("read", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(10));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, read_alias_with_no_data)
{
    InvokeCommand("r", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '\r', '\n' } }), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestHardwareTerminal, read_alias_with_multiple_samples)
{
    std::array<uint16_t, 5> sampleData1 = { 1000, 1500, 2000, 2500, 3000 };
    std::array<uint16_t, 5> sampleData2 = { 1100, 1600, 2100, 2600, 3100 };
    std::array<uint16_t, 5> sampleData3 = { 900, 1400, 1900, 2400, 2900 };

    onAdcMeasurementDone(hal::AdcMultiChannel::Samples{ sampleData1 });
    onAdcMeasurementDone(hal::AdcMultiChannel::Samples{ sampleData2 });
    onAdcMeasurementDone(hal::AdcMultiChannel::Samples{ sampleData3 });

    InvokeCommand("r", [this]()
        {
            EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(10));
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
