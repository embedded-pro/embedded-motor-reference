#include "application/motors/DC/components/MotorController.hpp"
#include "application/motors/DC/components/Terminal.hpp"
#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/event/test_helper/EventDispatcherWithWeakPtrFixture.hpp"
#include "infra/util/ByteRange.hpp"
#include "infra/util/MemoryRange.hpp"
#include "infra/util/test_helper/MemoryRangeMatcher.hpp"
#include "infra/util/test_helper/MockHelpers.hpp"
#include "services/util/Terminal.hpp"
#include "gmock/gmock.h"
#include <optional>

namespace
{
    MATCHER_P(SpeedEq, expected, "is speed equal")
    {
        return std::abs(arg.Value() - expected.Value()) < 1e-5f;
    }

    MATCHER_P(OptionalFloatEq, expected, "is an optional with value approximately " + ::testing::PrintToString(expected))
    {
        return arg.has_value() && std::abs(arg.value() - expected.value()) < 1e-5f;
    }

    MATCHER(OptionalEmpty, "is an empty optional")
    {
        return !arg.has_value();
    }

    MATCHER_P2(OptionalFloatNear, expected, tolerance,
        "is an optional with value near " + ::testing::PrintToString(expected) +
            " (tolerance: " + ::testing::PrintToString(tolerance) + ")")
    {
        return arg.has_value() && std::abs(arg.value() - expected) <= tolerance;
    }

    class StreamWriterMock
        : public infra::StreamWriter
    {
    public:
        using StreamWriter::StreamWriter;

        MOCK_METHOD2(Insert, void(infra::ConstByteRange range, infra::StreamErrorPolicy& errorPolicy));
        MOCK_CONST_METHOD0(Available, std::size_t());
        MOCK_CONST_METHOD0(ConstructSaveMarker, std::size_t());
        MOCK_CONST_METHOD1(GetProcessedBytesSince, std::size_t(std::size_t marker));
        MOCK_METHOD1(SaveState, infra::ByteRange(std::size_t marker));
        MOCK_METHOD1(RestoreState, void(infra::ByteRange range));
        MOCK_METHOD1(Overwrite, infra::ByteRange(std::size_t marker));
    };

    class MotorControllerMock
        : public application::MotorController
    {
    public:
        MOCK_METHOD(void, AutoTune, (const infra::Function<void()>& onDone), (override));
        MOCK_METHOD(void, SetPidParameters, (std::optional<float> kp, std::optional<float> ki, std::optional<float> kd), (override));
        MOCK_METHOD(void, SetSpeed, (const RevPerMinute& speed), (override));
        MOCK_METHOD(void, Start, (), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };

    class TestMotorTerminal
        : public ::testing::Test
        , public infra::EventDispatcherWithWeakPtrFixture
    {
    public:
        ::testing::StrictMock<MotorControllerMock> motorControllerMock;
        ::testing::StrictMock<StreamWriterMock> streamWriterMock;
        infra::TextOutputStream::WithErrorPolicy stream{ streamWriterMock };
        services::TracerToStream tracer{ stream };
        ::testing::StrictMock<hal::SerialCommunicationMock> communication;
        infra::Execute execute{ [this]()
            {
                EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '>', ' ' } }), testing::_));
            } };
        services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<> terminalWithCommands{ communication, tracer };
        services::TerminalWithStorage::WithMaxSize<10> terminal{ terminalWithCommands, tracer };
        application::TerminalInteractor terminalInteractor{ terminal, motorControllerMock };

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

TEST_F(TestMotorTerminal, set_auto_tune)
{
    InvokeCommand("auto_tune", [this]()
        {
            EXPECT_CALL(motorControllerMock, AutoTune(::testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, set_pid_parameters)
{
    std::optional<float> kp{ 1.1111f };
    std::optional<float> ki{ 2.2222f };
    std::optional<float> kd{ 3.3333f };

    InvokeCommand("set_pid 1.1111 2.2222 3.3333", [this, kp, ki, kd]()
        {
            EXPECT_CALL(motorControllerMock, SetPidParameters(OptionalFloatEq(kp), OptionalFloatEq(ki), OptionalFloatEq(kd)));
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, set_pid_parameters_with_invalid_kp)
{
    InvokeCommand("set_pid abc 2.2222 3.3333", [this]()
        {
            ::testing::InSequence _;

            std::string header{ "ERROR: " };
            std::string payload{ "invalid value. It should be a float." };
            std::string newline{ "\r\n" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, set_pid_parameters_with_invalid_ki)
{
    InvokeCommand("set_pid 2.2222 abc 3.3333", [this]()
        {
            ::testing::InSequence _;

            std::string header{ "ERROR: " };
            std::string payload{ "invalid value. It should be a float." };
            std::string newline{ "\r\n" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, set_pid_parameters_with_invalid_kd)
{
    InvokeCommand("set_pid 2.2222 3.3333 abc", [this]()
        {
            ::testing::InSequence _;

            std::string header{ "ERROR: " };
            std::string payload{ "invalid value. It should be a float." };
            std::string newline{ "\r\n" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, set_speed)
{
    application::MotorController::RevPerMinute speed{ 4.444f };

    InvokeCommand("set_speed 4.444", [this, speed]()
        {
            EXPECT_CALL(motorControllerMock, SetSpeed(SpeedEq(speed)));
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, set_speed_invalid)
{
    InvokeCommand("set_speed abc", [this]()
        {
            ::testing::InSequence _;

            std::string header{ "ERROR: " };
            std::string payload{ "invalid value. It should be a float." };
            std::string newline{ "\r\n" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, set_speed_with_extra_args)
{
    application::MotorController::RevPerMinute speed{ 4.444f };

    InvokeCommand("set_speed 1.1 abc", [this, speed]()
        {
            ::testing::InSequence _;

            std::string header{ "ERROR: " };
            std::string payload{ "invalid number of arguments." };
            std::string newline{ "\r\n" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, start)
{
    InvokeCommand("start", [this]()
        {
            EXPECT_CALL(motorControllerMock, Start());
        });

    ExecuteAllActions();
}

TEST_F(TestMotorTerminal, stop)
{
    InvokeCommand("stop", [this]()
        {
            EXPECT_CALL(motorControllerMock, Stop());
        });

    ExecuteAllActions();
}
