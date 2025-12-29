#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/event/test_helper/EventDispatcherWithWeakPtrFixture.hpp"
#include "infra/util/ByteRange.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/test_helper/MockHelpers.hpp"
#include "services/util/Terminal.hpp"
#include "source/services/cli/TerminalPosition.hpp"
#include "gmock/gmock.h"
#include <cmath>
#include <optional>

namespace
{
    MATCHER_P(PositionEq, expected, "is position equal")
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
        : public services::FocPositionInteractor
    {
    public:
        MOCK_METHOD(void, AutoTune, (const infra::Function<void()>& onDone), (override));
        MOCK_METHOD(void, SetDQPidParameters, ((const std::pair<PidParameters, PidParameters>&)dqPidParams), (override));
        MOCK_METHOD(void, SetSpeedPidParameters, (const PidParameters& pidParameters), (override));
        MOCK_METHOD(void, SetPositionPidParameters, (const PidParameters& pidParameters), (override));
        MOCK_METHOD(void, SetPosition, (const foc::Radians& position), (override));
        MOCK_METHOD(void, Start, (), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };

    class TerminalPositionTest
        : public ::testing::Test
        , public infra::EventDispatcherWithWeakPtrFixture
    {
    public:
        ::testing::StrictMock<FocControllerMock> focControllerMock;
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
        services::TerminalFocPositionInteractor terminalInteractor{ terminal, focControllerMock };

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

TEST_F(TerminalPositionTest, auto_tune)
{
    InvokeCommand("auto_tune", [this]()
        {
            EXPECT_CALL(focControllerMock, AutoTune(::testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, auto_tune_alias)
{
    InvokeCommand("at", [this]()
        {
            EXPECT_CALL(focControllerMock, AutoTune(::testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, set_dq_pid)
{
    services::FocInteractor::PidParameters dParams{
        std::optional<float>(1.0f),
        std::optional<float>(0.765f),
        std::optional<float>(-0.56f)
    };
    services::FocInteractor::PidParameters qParams{
        std::optional<float>(0.5f),
        std::optional<float>(-0.35f),
        std::optional<float>(0.75f)
    };

    std::pair<services::FocInteractor::PidParameters,
        services::FocInteractor::PidParameters>
        expectedParams(dParams, qParams);

    InvokeCommand("sdqpid 1.0 0.765 -0.56 0.5 -0.35 0.75", [this, &expectedParams]()
        {
            EXPECT_CALL(focControllerMock, SetDQPidParameters(PidParamsEq(expectedParams.first, expectedParams.second)));
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, set_dq_pid_invalid_argument_count)
{
    InvokeCommand("set_dq_pid 1.0 0.765", [this]()
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

TEST_F(TerminalPositionTest, set_dq_pid_invalid_d_kp)
{
    InvokeCommand("set_dq_pid abc 0.765 -0.56 0.5 -0.35 0.75", [this]()
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

TEST_F(TerminalPositionTest, set_dq_pid_invalid_d_ki)
{
    InvokeCommand("set_dq_pid 1.0 abc -0.56 0.5 -0.35 0.75", [this]()
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

TEST_F(TerminalPositionTest, set_dq_pid_invalid_d_kd)
{
    InvokeCommand("set_dq_pid 1.0 0.765 abc 0.5 -0.35 0.75", [this]()
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

TEST_F(TerminalPositionTest, set_dq_pid_invalid_q_kp)
{
    InvokeCommand("set_dq_pid 1.0 0.765 -0.56 abc -0.35 0.75", [this]()
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

TEST_F(TerminalPositionTest, set_dq_pid_invalid_q_ki)
{
    InvokeCommand("set_dq_pid 1.0 0.765 -0.56 0.5 abc 0.75", [this]()
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

TEST_F(TerminalPositionTest, set_dq_pid_invalid_q_kd)
{
    InvokeCommand("set_dq_pid 1.0 0.765 -0.56 0.5 -0.35 abc", [this]()
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

TEST_F(TerminalPositionTest, set_speed_pid)
{
    services::FocInteractor::PidParameters pid{
        std::optional<float>(0.5f),
        std::optional<float>(0.1f),
        std::optional<float>(0.01f)
    };

    InvokeCommand("sspid 0.5 0.1 0.01", [this, &pid]()
        {
            EXPECT_CALL(focControllerMock, SetSpeedPidParameters(PidParamEq(pid)));
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, set_speed_pid_invalid_argument_count)
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

TEST_F(TerminalPositionTest, set_speed_pid_invalid_kp)
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

TEST_F(TerminalPositionTest, set_speed_pid_invalid_ki)
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

TEST_F(TerminalPositionTest, set_speed_pid_invalid_kd)
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

TEST_F(TerminalPositionTest, set_pos_pid)
{
    services::FocInteractor::PidParameters pid{
        std::optional<float>(0.5f),
        std::optional<float>(0.1f),
        std::optional<float>(0.01f)
    };

    InvokeCommand("sppid 0.5 0.1 0.01", [this, &pid]()
        {
            EXPECT_CALL(focControllerMock, SetPositionPidParameters(PidParamEq(pid)));
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, set_pos_pid_invalid_argument_count)
{
    InvokeCommand("set_pos_pid 0.5 0.1", [this]()
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

TEST_F(TerminalPositionTest, set_pos_pid_invalid_kp)
{
    InvokeCommand("set_pos_pid abc 0.1 0.01", [this]()
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

TEST_F(TerminalPositionTest, set_pos_pid_invalid_ki)
{
    InvokeCommand("set_pos_pid 0.5 abc 0.01", [this]()
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

TEST_F(TerminalPositionTest, set_pos_pid_invalid_kd)
{
    InvokeCommand("set_pos_pid 0.5 0.1 abc", [this]()
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

TEST_F(TerminalPositionTest, set_position)
{
    InvokeCommand("spr 2.5", [this]()
        {
            EXPECT_CALL(focControllerMock, SetPosition(testing::A<const foc::Radians&>()))
                .WillOnce(testing::Invoke([](const foc::Radians& p)
                    {
                        EXPECT_NEAR(p.Value(), 2.5f, 1e-5f);
                    }));
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, set_position_invalid_argument_count)
{
    InvokeCommand("set_position 2.5 3.5", [this]()
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

TEST_F(TerminalPositionTest, set_position_invalid_value)
{
    InvokeCommand("set_position abc", [this]()
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

TEST_F(TerminalPositionTest, start)
{
    InvokeCommand("start", [this]()
        {
            EXPECT_CALL(focControllerMock, Start());
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, start_alias)
{
    InvokeCommand("sts", [this]()
        {
            EXPECT_CALL(focControllerMock, Start());
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, stop)
{
    InvokeCommand("stop", [this]()
        {
            EXPECT_CALL(focControllerMock, Stop());
        });

    ExecuteAllActions();
}

TEST_F(TerminalPositionTest, stop_alias)
{
    InvokeCommand("stp", [this]()
        {
            EXPECT_CALL(focControllerMock, Stop());
        });

    ExecuteAllActions();
}
