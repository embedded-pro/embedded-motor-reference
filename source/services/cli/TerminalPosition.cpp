#include "source/services/cli/TerminalPosition.hpp"
#include "infra/util/Tokenizer.hpp"
#include "source/services/cli/TerminalHelper.hpp"

namespace services
{
    TerminalFocPositionInteractor::TerminalFocPositionInteractor(services::TerminalWithStorage& terminal, foc::Volts vdc, foc::ControllerBase& foc, foc::PositionController& position)
        : TerminalFocBaseInteractor(terminal, vdc, foc)
        , vdc(vdc)
        , foc(position)
    {
        terminal.AddCommand({ { "set_speed_pid", "sspid", "Set speed PID parameters. set_speed_pid <kp> <ki> <kd>. Ex: sspid 1.0 0.765 -0.56" },
            [this](const auto& params)
            {
                this->Terminal().ProcessResult(SetSpeedPid(params));
            } });

        terminal.AddCommand({ { "set_pos_pid", "sppid", "Set position PID parameters. set_pos_pid <kp> <ki> <kd>. Ex: sppid 1.0 0.765 -0.56" },
            [this](const auto& params)
            {
                this->Terminal().ProcessResult(SetPositionPid(params));
            } });

        terminal.AddCommand({ { "set_position", "spr", "Set position in rad. set_position <position>. Ex: spr 20.0" },
            [this](const auto& params)
            {
                this->Terminal().ProcessResult(SetPosition(params));
            } });
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

        auto pid = controllers::PidTunings<float>{ (*kp), (*ki), (*kd) };

        foc.SetSpeedTunings(vdc, pid);
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

        auto pid = controllers::PidTunings<float>{ (*kp), (*ki), (*kd) };

        foc.SetPositionTunings(vdc, pid);
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

        foc.SetPoint(position);
        return TerminalFocPositionInteractor::StatusWithMessage();
    }
}
