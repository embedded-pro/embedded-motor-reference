#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/event/test_helper/EventDispatcherWithWeakPtrFixture.hpp"
#include "infra/util/ByteRange.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/test_helper/MockHelpers.hpp"
#include "services/util/Terminal.hpp"
#include "source/services/cli/TerminalSpeed.hpp"
#include "gmock/gmock.h"
#include <cmath>
#include <optional>

namespace
{
    MATCHER_P(SpeedEq, expected, "is speed equal")
    {
        return std::abs(arg.Value() - expected.Value()) < 1e-5f;
    }

    MATCHER_P2(PidParamsEq, dParams, qParams, "is FOC PID parameters equal")
    {
        const auto& d = arg.first;
        const auto& q = arg.second;

        return (!d.kp.has_value() && !dParams.kp.has_value() ||
                   (d.kp.has_value() && dParams.kp.has_value() && std::abs(*d.kp - *dParams.kp) < 1e-5f)) &&
               (!d.ki.has_value() && !dParams.ki.has_value() ||
                   (d.ki.has_value() && dParams.ki.has_value() && std::abs(*d.ki - *dParams.ki) < 1e-5f)) &&
               (!d.kd.has_value() && !dParams.kd.has_value() ||
                   (d.kd.has_value() && dParams.kd.has_value() && std::abs(*d.kd - *dParams.kd) < 1e-5f)) &&
               (!q.kp.has_value() && !qParams.kp.has_value() ||
                   (q.kp.has_value() && qParams.kp.has_value() && std::abs(*q.kp - *qParams.kp) < 1e-5f)) &&
               (!q.ki.has_value() && !qParams.ki.has_value() ||
                   (q.ki.has_value() && qParams.ki.has_value() && std::abs(*q.ki - *qParams.ki) < 1e-5f)) &&
               (!q.kd.has_value() && !qParams.kd.has_value() ||
                   (q.kd.has_value() && qParams.kd.has_value() && std::abs(*q.kd - *qParams.kd) < 1e-5f));
    }

    MATCHER_P(PidParamEq, expected, "is PID parameters equal")
    {
        return (!arg.kp.has_value() && !expected.kp.has_value() ||
                   (arg.kp.has_value() && expected.kp.has_value() && std::abs(*arg.kp - *expected.kp) < 1e-5f)) &&
               (!arg.ki.has_value() && !expected.ki.has_value() ||
                   (arg.ki.has_value() && expected.ki.has_value() && std::abs(*arg.ki - *expected.ki) < 1e-5f)) &&
               (!arg.kd.has_value() && !expected.kd.has_value() ||
                   (arg.kd.has_value() && expected.kd.has_value() && std::abs(*arg.kd - *expected.kd) < 1e-5f));
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

    class FocControllerMock
        : public services::FocInteractor
    {
    public:
        MOCK_METHOD(void, AutoTune, (const infra::Function<void()>& onDone), (override));
        MOCK_METHOD(void, SetDQPidParameters, ((const std::pair<services::PidParameters, services::PidParameters>&)dqPidParams), (override));
        MOCK_METHOD(void, Start, (), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };

    class FocSpeedControllerMock
        : public services::FocSpeedInteractor
    {
    public:
        MOCK_METHOD(void, SetSpeed, (const foc::RevPerMinute& speed), (override));
        MOCK_METHOD(void, SetSpeed, (const foc::RadiansPerSecond& speed), (override));
        MOCK_METHOD(void, SetSpeedPidParameters, (const services::PidParameters& pidParameters), (override));
    };

    class TerminalSpeedTest
        : public ::testing::Test
        , public infra::EventDispatcherWithWeakPtrFixture
    {
    public:
        ::testing::StrictMock<FocControllerMock> focControllerMock;
        ::testing::StrictMock<FocSpeedControllerMock> focSpeedControllerMock;
        ::testing::StrictMock<StreamWriterMock> streamWriterMock;
        infra::TextOutputStream::WithErrorPolicy stream{ streamWriterMock };
        services::TracerToStream tracer{ stream };
        ::testing::StrictMock<hal::SerialCommunicationMock> communication;
        infra::Execute execute{ [this]()
            {
                EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>{ { '>', ' ' } }), testing::_));
            } };
        services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<128, 5> terminalWithCommands{ communication, tracer };
        services::TerminalWithStorage::WithMaxSize<10> terminal{ terminalWithCommands, tracer };
        services::TerminalFocSpeedInteractor terminalInteractor{ terminal, focControllerMock, focSpeedControllerMock };

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

TEST_F(TerminalSpeedTest, set_speed_pid)
{
    services::PidParameters pid{
        std::optional<float>(0.5f),
        std::optional<float>(0.1f),
        std::optional<float>(0.01f)
    };

    InvokeCommand("sspid 0.5 0.1 0.01", [this, &pid]()
        {
            EXPECT_CALL(focSpeedControllerMock, SetSpeedPidParameters(PidParamEq(pid)));
        });

    ExecuteAllActions();
}

TEST_F(TerminalSpeedTest, set_speed_pid_invalid_argument_count)
{
    InvokeCommand("set_speed_pid 0.5 0.1", [this]()
        {
            ::testing::InSequence _;

            std::string header{ "ERROR: " };
            std::string payload{ "invalid number of arguments" };
            std::string newline{ "\r\n" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalSpeedTest, set_speed_pid_invalid_kp)
{
    InvokeCommand("set_speed_pid abc 0.1 0.01", [this]()
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

TEST_F(TerminalSpeedTest, set_speed_pid_invalid_ki)
{
    InvokeCommand("set_speed_pid 0.5 abc 0.01", [this]()
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

TEST_F(TerminalSpeedTest, set_speed_pid_invalid_kd)
{
    InvokeCommand("set_speed_pid 0.5 0.1 abc", [this]()
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

TEST_F(TerminalSpeedTest, set_speed)
{
    InvokeCommand("ss 2.5", [this]()
        {
            EXPECT_CALL(focSpeedControllerMock, SetSpeed(testing::A<const foc::RadiansPerSecond&>()))
                .WillOnce(testing::Invoke([](const foc::RadiansPerSecond& s)
                    {
                        EXPECT_NEAR(s.Value(), 2.5f, 1e-5f);
                    }));
        });

    ExecuteAllActions();
}

TEST_F(TerminalSpeedTest, set_speed_invalid_argument_count)
{
    InvokeCommand("set_speed 2.5 3.5", [this]()
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

TEST_F(TerminalSpeedTest, set_speed_invalid_value)
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
