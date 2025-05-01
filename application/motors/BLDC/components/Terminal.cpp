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
    TerminalInteractor::TerminalInteractor(services::TerminalWithStorage& terminal, FieldOrientedControllerInteractor& focInteractor)
        : terminal(terminal)
        , focInteractor(focInteractor)
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

        terminal.AddCommand({ { "set_torque", "st", "Set torque. set_torque <torque>. Ex: st 20.0" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(SetTorque(params));
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
        focInteractor.AutoTune(infra::emptyFunction);
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::SetFocPid(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 6)
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

        auto dPid = FieldOrientedControllerInteractor::PidParameters{
            std::optional<float>(*dkp),
            std::optional<float>(*dki),
            std::optional<float>(*dkd)
        };
        auto qPid = FieldOrientedControllerInteractor::PidParameters{
            std::optional<float>(*qkp),
            std::optional<float>(*qki),
            std::optional<float>(*qkd)
        };

        focInteractor.SetDQPidParameters(std::make_pair(dPid, qPid));
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::Start()
    {
        focInteractor.Start();
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::SetTorque(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 1)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments." };

        auto t = ParseInput(tokenizer.Token(0));
        if (!t)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        FieldOrientedControllerInteractor::Torque torque(*t);

        focInteractor.SetTorque(torque);
        return TerminalInteractor::StatusWithMessage();
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::Stop()
    {
        focInteractor.Stop();
        return TerminalInteractor::StatusWithMessage();
    }
}
