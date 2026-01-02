#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/event/test_helper/EventDispatcherWithWeakPtrFixture.hpp"
#include "infra/util/ByteRange.hpp"
#include "infra/util/test_helper/MockHelpers.hpp"
#include "services/util/Terminal.hpp"
#include "source/services/parameter_identification/MotorIdentification.hpp"
#include "source/services/parameter_identification/TerminalMotorIdentification.hpp"
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

    class MotorIdentificationMock
        : public services::MotorIdentification
    {
    public:
        MOCK_METHOD2(GetResistance, void(const ResistanceConfig& config, const infra::Function<void(std::optional<foc::Ohm>)>& onDone));
        MOCK_METHOD2(GetInductance, void(const InductanceConfig& config, const infra::Function<void(std::optional<foc::Henry>)>& onDone));
        MOCK_METHOD2(GetNumberOfPolePairs, void(const PolePairsConfig& config, const infra::Function<void(std::optional<std::size_t>)>& onDone));
    };

    class TerminalMotorIdentificationTest
        : public ::testing::Test
        , public infra::EventDispatcherWithWeakPtrFixture
    {
    public:
        ::testing::StrictMock<MotorIdentificationMock> identificationMock;
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
        services::TerminalMotorIdentification terminalIdentification{ terminal, tracer, identificationMock };

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

TEST_F(TerminalMotorIdentificationTest, estimate_resistance_calls_identification)
{
    InvokeCommand("estimate_resistance", [this]()
        {
            EXPECT_CALL(identificationMock, GetResistance(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_resistance_short_command)
{
    InvokeCommand("estr", [this]()
        {
            EXPECT_CALL(identificationMock, GetResistance(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_resistance_failure)
{
    InvokeCommand("estr", [this]()
        {
            EXPECT_CALL(identificationMock, GetResistance(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_inductance_calls_identification)
{
    InvokeCommand("estimate_inductance", [this]()
        {
            EXPECT_CALL(identificationMock, GetInductance(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_inductance_short_command)
{
    InvokeCommand("estl", [this]()
        {
            EXPECT_CALL(identificationMock, GetInductance(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_inductance_failure)
{
    InvokeCommand("estl", [this]()
        {
            EXPECT_CALL(identificationMock, GetInductance(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_pole_pairs_calls_identification)
{
    InvokeCommand("estimate_pole_pairs", [this]()
        {
            EXPECT_CALL(identificationMock, GetNumberOfPolePairs(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_pole_pairs_short_command)
{
    InvokeCommand("estpp", [this]()
        {
            EXPECT_CALL(identificationMock, GetNumberOfPolePairs(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_pole_pairs_failure)
{
    InvokeCommand("estpp", [this]()
        {
            EXPECT_CALL(identificationMock, GetNumberOfPolePairs(testing::_, testing::_));
        });

    ExecuteAllActions();
}
