#include "application/motors/dc/components/Terminal.hpp"
#include "infra/stream/StringInputStream.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/Tokenizer.hpp"
#include "services/util/TerminalWithStorage.hpp"

namespace
{
    std::optional<float> ParseInput(const infra::BoundedConstString& data)
    {
        float value = 0.0f;
        infra::StringInputStream stream(data, infra::softFail);
        stream >> value;

        if (!stream.ErrorPolicy().Failed())
            return std::make_optional(value);
        else
            return {};
    }
}

namespace application
{
    TerminalInteractor::TerminalInteractor(services::TerminalWithStorage& terminal, MotorPidController& motorController)
        : terminal(terminal)
        , motorController(motorController)
    {
        terminal.AddCommand({ { "auto_tune", "at", "Run auto tune" },
            [this](const auto&)
            {
                this->terminal.ProcessResult(AutoTune());
            } });

        terminal.AddCommand({ { "set_pid", "spid", "Set PID parameters. set_pid <kp> <ki> <kd>. Ex: spid 1.0 0.765 -0.56" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(SetKpKiKd(params));
            } });

        terminal.AddCommand({ { "set_speed", "ss", "Set speed. set_speed <speed>. Ex: ss 20.0" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(SetSpeed(params));
            } });

        terminal.AddCommand({ { "start", "sts", "Start system. start. Ex: start" },
            [this](const auto&)
            {
                this->terminal.ProcessResult(Start());
            } });

        terminal.AddCommand({ { "stop", "stp", "Stop system. stop. Ex: stop" },
            [this](const auto&)
            {
                this->terminal.ProcessResult(Stop());
            } });
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::AutoTune()
    {
        motorController.AutoTune(infra::emptyFunction);
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::SetKpKiKd(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 3)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments" };

        auto kp = ParseInput(tokenizer.Token(0));
        if (!kp)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto ki = ParseInput(tokenizer.Token(1));
        if (!ki)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto kd = ParseInput(tokenizer.Token(2));
        if (!kd)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        motorController.SetPidParameters(kp, ki, kd);
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::SetSpeed(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 1)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments." };

        auto speed = ParseInput(tokenizer.Token(0));
        if (!speed)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        MotorPidController::RevPerMinute revPerMinute(*speed);
        motorController.SetSpeed(revPerMinute);
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::Start()
    {
        motorController.Start();
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::Stop()
    {
        motorController.Stop();
        return TerminalInteractor::StatusWithMessage();
    }
}
