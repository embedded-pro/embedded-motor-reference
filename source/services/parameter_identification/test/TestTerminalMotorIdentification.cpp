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

TEST_F(TerminalMotorIdentificationTest, estimate_resistance_successful_callback)
{
    infra::Function<void(std::optional<foc::Ohm>)> capturedCallback;

    InvokeCommand("estr", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetResistance(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string prefix{ "Estimated Resistance: " };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(prefix.begin(), prefix.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(1));

    capturedCallback(foc::Ohm{ 1.5f });
    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_resistance_failure_callback)
{
    infra::Function<void(std::optional<foc::Ohm>)> capturedCallback;

    InvokeCommand("estimate_resistance", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetResistance(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string message{ "Resistance estimation failed." };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(message.begin(), message.end())), testing::_));

    capturedCallback(std::nullopt);
    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_resistance_verifies_default_config)
{
    services::MotorIdentification::ResistanceConfig capturedConfig;

    InvokeCommand("estr", [this, &capturedConfig]()
        {
            EXPECT_CALL(identificationMock, GetResistance(testing::_, testing::_))
                .WillOnce(testing::SaveArg<0>(&capturedConfig));
        });

    ExecuteAllActions();

    services::MotorIdentification::ResistanceConfig defaultConfig;
    EXPECT_EQ(capturedConfig.testVoltagePercent, defaultConfig.testVoltagePercent);
    EXPECT_EQ(capturedConfig.sampleCount, defaultConfig.sampleCount);
    EXPECT_EQ(capturedConfig.minCurrent, defaultConfig.minCurrent);
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

TEST_F(TerminalMotorIdentificationTest, estimate_inductance_successful_callback)
{
    infra::Function<void(std::optional<foc::Henry>)> capturedCallback;

    InvokeCommand("estl", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetInductance(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string prefix{ "Estimated Inductance: " };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(prefix.begin(), prefix.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(1));

    capturedCallback(foc::Henry{ 0.001f });
    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_inductance_failure_callback)
{
    infra::Function<void(std::optional<foc::Henry>)> capturedCallback;

    InvokeCommand("estimate_inductance", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetInductance(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string message{ "Inductance estimation failed." };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(message.begin(), message.end())), testing::_));

    capturedCallback(std::nullopt);
    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_inductance_verifies_default_config)
{
    services::MotorIdentification::InductanceConfig capturedConfig;

    InvokeCommand("estl", [this, &capturedConfig]()
        {
            EXPECT_CALL(identificationMock, GetInductance(testing::_, testing::_))
                .WillOnce(testing::SaveArg<0>(&capturedConfig));
        });

    ExecuteAllActions();

    services::MotorIdentification::InductanceConfig defaultConfig;
    EXPECT_EQ(capturedConfig.testVoltagePercent, defaultConfig.testVoltagePercent);
    EXPECT_EQ(capturedConfig.resistance, defaultConfig.resistance);
    EXPECT_EQ(capturedConfig.samplingFrequency, defaultConfig.samplingFrequency);
    EXPECT_EQ(capturedConfig.minCurrentChange, defaultConfig.minCurrentChange);
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

TEST_F(TerminalMotorIdentificationTest, estimate_pole_pairs_successful_callback)
{
    infra::Function<void(std::optional<std::size_t>)> capturedCallback;

    InvokeCommand("estpp", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetNumberOfPolePairs(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string prefix{ "Estimated Pole Pairs: " };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(prefix.begin(), prefix.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(1));

    capturedCallback(7);
    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_pole_pairs_failure_callback)
{
    infra::Function<void(std::optional<std::size_t>)> capturedCallback;

    InvokeCommand("estimate_pole_pairs", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetNumberOfPolePairs(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string message{ "Pole pairs estimation failed." };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(message.begin(), message.end())), testing::_));

    capturedCallback(std::nullopt);
    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_pole_pairs_verifies_default_config)
{
    services::MotorIdentification::PolePairsConfig capturedConfig;

    InvokeCommand("estpp", [this, &capturedConfig]()
        {
            EXPECT_CALL(identificationMock, GetNumberOfPolePairs(testing::_, testing::_))
                .WillOnce(testing::SaveArg<0>(&capturedConfig));
        });

    ExecuteAllActions();

    services::MotorIdentification::PolePairsConfig defaultConfig;
    EXPECT_EQ(capturedConfig.testVoltagePercent, defaultConfig.testVoltagePercent);
    EXPECT_EQ(capturedConfig.electricalRevolutions, defaultConfig.electricalRevolutions);
    EXPECT_EQ(capturedConfig.minMechanicalRotation, defaultConfig.minMechanicalRotation);
}

TEST_F(TerminalMotorIdentificationTest, multiple_commands_can_be_invoked_sequentially)
{
    InvokeCommand("estr", [this]()
        {
            EXPECT_CALL(identificationMock, GetResistance(testing::_, testing::_));
        });

    ExecuteAllActions();

    InvokeCommand("estl", [this]()
        {
            EXPECT_CALL(identificationMock, GetInductance(testing::_, testing::_));
        });

    ExecuteAllActions();

    InvokeCommand("estpp", [this]()
        {
            EXPECT_CALL(identificationMock, GetNumberOfPolePairs(testing::_, testing::_));
        });

    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_resistance_with_zero_result)
{
    infra::Function<void(std::optional<foc::Ohm>)> capturedCallback;

    InvokeCommand("estr", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetResistance(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string prefix{ "Estimated Resistance: " };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(prefix.begin(), prefix.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(1));

    capturedCallback(foc::Ohm{ 0.0f });
    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_inductance_with_zero_result)
{
    infra::Function<void(std::optional<foc::Henry>)> capturedCallback;

    InvokeCommand("estl", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetInductance(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string prefix{ "Estimated Inductance: " };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(prefix.begin(), prefix.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(1));

    capturedCallback(foc::Henry{ 0.0f });
    ExecuteAllActions();
}

TEST_F(TerminalMotorIdentificationTest, estimate_pole_pairs_with_zero_result)
{
    infra::Function<void(std::optional<std::size_t>)> capturedCallback;

    InvokeCommand("estpp", [this, &capturedCallback]()
        {
            EXPECT_CALL(identificationMock, GetNumberOfPolePairs(testing::_, testing::_))
                .WillOnce(testing::SaveArg<1>(&capturedCallback));
        });

    ExecuteAllActions();

    ::testing::InSequence _;
    std::string newline{ "\r\n" };
    std::string prefix{ "Estimated Pole Pairs: " };
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(newline.begin(), newline.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(infra::CheckByteRangeContents(std::vector<uint8_t>(prefix.begin(), prefix.end())), testing::_));
    EXPECT_CALL(streamWriterMock, Insert(testing::_, testing::_)).Times(testing::AtLeast(1));

    capturedCallback(0);
    ExecuteAllActions();
}
