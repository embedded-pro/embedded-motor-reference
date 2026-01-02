#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/event/test_helper/EventDispatcherWithWeakPtrFixture.hpp"
#include "infra/util/ByteRange.hpp"
#include "infra/util/test_helper/MockHelpers.hpp"
#include "services/util/Terminal.hpp"
#include "source/services/alignment/MotorAlignment.hpp"
#include "source/services/alignment/TerminalMotorAlignment.hpp"
#include "gmock/gmock.h"

namespace
{
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

    class MotorAlignmentMock
        : public services::MotorAlignment
    {
    public:
        MOCK_METHOD3(ForceAlignment, void(std::size_t polePairs, const AlignmentConfig& config, const infra::Function<void(std::optional<foc::Radians>)>& onDone));
    };

    class TerminalMotorAlignmentTest
        : public ::testing::Test
        , public infra::EventDispatcherWithWeakPtrFixture
    {
    public:
        ::testing::StrictMock<MotorAlignmentMock> alignmentMock;
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
        services::TerminalMotorAlignment terminalAlignment{ terminal, tracer, alignmentMock };

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

TEST_F(TerminalMotorAlignmentTest, force_alignment)
{
    InvokeCommand("force_alignment 7", [this]()
        {
            EXPECT_CALL(alignmentMock, ForceAlignment(testing::_, testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorAlignmentTest, force_alignment_short_command)
{
    InvokeCommand("fa 4", [this]()
        {
            EXPECT_CALL(alignmentMock, ForceAlignment(testing::_, testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorAlignmentTest, force_alignment_invalid_argument_count)
{
    InvokeCommand("force_alignment", [this]()
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

TEST_F(TerminalMotorAlignmentTest, force_alignment_invalid_pole_pairs)
{
    InvokeCommand("fa abc", [this]()
        {
            ::testing::InSequence _;

            std::string header{ "ERROR: " };
            std::string payload{ "invalid value. It should be an integer." };
            std::string newline{ "\r\n" };

            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(header.begin(), header.end())), testing::_));
            EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(payload.begin(), payload.end())), testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorAlignmentTest, force_alignment_too_many_arguments)
{
    InvokeCommand("fa 7 extra", [this]()
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
