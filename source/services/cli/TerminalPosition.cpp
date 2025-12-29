#include "source/services/cli/TerminalPosition.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/Tokenizer.hpp"
#include "services/util/TerminalWithStorage.hpp"
#include "source/services/cli/TerminalHelper.hpp"

namespace services
{
    TerminalFocPositionInteractor::TerminalFocPositionInteractor(services::TerminalWithStorage& terminal, FocPositionInteractor& foc)
        : terminal(terminal)
        , foc(foc)
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

        terminal.AddCommand({ { "set_pos_pid", "sppid", "Set position PID parameters. set_pos_pid <kp> <ki> <kd>. Ex: sppid 1.0 0.765 -0.56" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(SetPositionPid(params));
            } });

        terminal.AddCommand({ { "set_position", "spr", "Set position in rad. set_position <position>. Ex: spr 20.0" },
            [this](const auto& params)
            {
                this->terminal.ProcessResult(SetPosition(params));
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

    TerminalFocPositionInteractor::StatusWithMessage TerminalFocPositionInteractor::AutoTune()
    {
        foc.AutoTune(infra::emptyFunction);
        return TerminalFocPositionInteractor::StatusWithMessage();
    }

    TerminalFocPositionInteractor::StatusWithMessage TerminalFocPositionInteractor::SetFocPid(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 6)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments" };

        auto dkp = ParseInput(tokenizer.Token(0));
        if (!dkp.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto dki = ParseInput(tokenizer.Token(1));
        if (!dki.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto dkd = ParseInput(tokenizer.Token(2));
        if (!dkd.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto qkp = ParseInput(tokenizer.Token(3));
        if (!qkp.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto qki = ParseInput(tokenizer.Token(4));
        if (!qki.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto qkd = ParseInput(tokenizer.Token(5));
        if (!qkd.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto dPid = FocInteractor::PidParameters{
            std::optional<float>(*dkp),
            std::optional<float>(*dki),
            std::optional<float>(*dkd)
        };
        auto qPid = FocInteractor::PidParameters{
            std::optional<float>(*qkp),
            std::optional<float>(*qki),
            std::optional<float>(*qkd)
        };

        foc.SetDQPidParameters(std::make_pair(dPid, qPid));
        return TerminalFocPositionInteractor::StatusWithMessage();
    }

    TerminalFocPositionInteractor::StatusWithMessage TerminalFocPositionInteractor::SetSpeedPid(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 3)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments" };

        auto kp = ParseInput(tokenizer.Token(0));
        if (!kp.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };
        auto ki = ParseInput(tokenizer.Token(1));
        if (!ki.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };
        auto kd = ParseInput(tokenizer.Token(2));
        if (!kd.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto pid = FocInteractor::PidParameters{
            std::optional<float>(*kp),
            std::optional<float>(*ki),
            std::optional<float>(*kd)
        };

        foc.SetSpeedPidParameters(pid);
        return TerminalFocPositionInteractor::StatusWithMessage();
    }

    TerminalFocPositionInteractor::StatusWithMessage TerminalFocPositionInteractor::SetPositionPid(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 3)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments" };

        auto kp = ParseInput(tokenizer.Token(0));
        if (!kp.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };
        auto ki = ParseInput(tokenizer.Token(1));
        if (!ki.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };
        auto kd = ParseInput(tokenizer.Token(2));
        if (!kd.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        auto pid = FocInteractor::PidParameters{
            std::optional<float>(*kp),
            std::optional<float>(*ki),
            std::optional<float>(*kd)
        };

        foc.SetPositionPidParameters(pid);
        return TerminalFocPositionInteractor::StatusWithMessage();
    }

    TerminalFocPositionInteractor::StatusWithMessage TerminalFocPositionInteractor::Start()
    {
        foc.Start();
        return TerminalFocPositionInteractor::StatusWithMessage();
    }

    TerminalFocPositionInteractor::StatusWithMessage TerminalFocPositionInteractor::SetPosition(const infra::BoundedConstString& input)
    {
        infra::Tokenizer tokenizer(input, ' ');

        if (tokenizer.Size() != 1)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments." };

        auto t = ParseInput(tokenizer.Token(0));
        if (!t.has_value())
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float." };

        foc::Radians position(*t);

        foc.SetPosition(position);
        return TerminalFocPositionInteractor::StatusWithMessage();
    }

    TerminalFocPositionInteractor::StatusWithMessage TerminalFocPositionInteractor::Stop()
    {
        foc.Stop();
        return TerminalFocPositionInteractor::StatusWithMessage();
    }
}
