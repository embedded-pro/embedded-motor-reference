#include "application/motors/BLDC/components/Terminal.hpp"
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
    TerminalInteractor::TerminalInteractor(services::TerminalWithStorage& terminal, application::FocController& focController)
        : terminal(terminal)
        , focController(focController)
    {
        terminal.AddCommand({ { "auto_tune", "at", "Run auto tune" },
            [this](const auto&)
            {
                this->terminal.ProcessResult(AutoTune());
            } });

        terminal.AddCommand({ { "set_dq_pid", "sdqpid", "Set D and Q PID parameters. set_dq_pid <kp> <ki> <kd> <kp> <ki> <kd>. Ex: sdqpid 1.0 0.765 -0.56 0.5 -0.35 0.75" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(SetFocPid(params));
            } });

        terminal.AddCommand({ { "set_speed_pid", "sspid", "Set speed PID parameters. set_speed_pid <kp> <ki> <kd>. Ex: sspid 1.0 0.765 -0.56" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(SetSpeedPid(params));
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
        focController.AutoTune(infra::emptyFunction);
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::SetFocPid(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 3)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments" };

        auto dkp = ParseInput(tokenizer.Token(0));
        if (!dkp)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto dki = ParseInput(tokenizer.Token(1));
        if (!dki)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto dkd = ParseInput(tokenizer.Token(2));
        if (!dkd)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto qkp = ParseInput(tokenizer.Token(3));
        if (!qkp)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto qki = ParseInput(tokenizer.Token(4));
        if (!qki)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto qkd = ParseInput(tokenizer.Token(5));
        if (!qkd)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto dPid = FocController::PidFocParameters{ *dkp, *dki, *dkd };
        auto qPid = FocController::PidFocParameters{ *qkp, *qki, *qkd };

        focController.SetDQPidParameters(std::make_pair(dPid, qPid));
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::SetSpeedPid(const infra::BoundedConstString& input)
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

        focController.SetSpeedPidParameters(*kp, *ki, *kd);
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

        FocController::RevPerMinute revPerMinute(*speed);
        focController.SetSpeed(revPerMinute);
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::Start()
    {
        focController.Start();
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::Stop()
    {
        focController.Stop();
        return TerminalInteractor::StatusWithMessage();
    }
}
