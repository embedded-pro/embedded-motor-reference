#include "application/motors/hardware_test/components/Terminal.hpp"
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
    TerminalInteractor::TerminalInteractor(services::TerminalWithStorage& terminal, application::HardwareFactory& hardware)
        : terminal(terminal)
        , hardware(hardware)
    {
        terminal.AddCommand({ { "start", "sts", "Start pwm. start. Ex: start" },
            [this](const auto&)
            {
                this->terminal.ProcessResult(Start());
            } });

        terminal.AddCommand({ { "stop", "stp", "Stop pwm. stop. Ex: stop" },
            [this](const auto&)
            {
                this->terminal.ProcessResult(Stop());
            } });

        terminal.AddCommand({ { "read", "r", "Read adc with sample time. Ex: read" },
            [this](const auto&)
            {
                this->terminal.ProcessResult(ReadAdcWithSampleTime());
            } });

        terminal.AddCommand({ { "duty", "d", "Set pwm duty. Ex: duty 0 10 25" },
            [this](const infra::BoundedConstString& param)
            {
                this->terminal.ProcessResult(SetPwmDuty(param));
            } });

        terminal.AddCommand({ { "pwm", "p", "Configure pwm [dead_time ns [500; 2000]] [frequency Hz [10000; 20000]]. Ex: pwm 500 10000" },
            [this](const infra::BoundedConstString& param)
            {
                this->terminal.ProcessResult(ConfigurePwm(param));
            } });

        terminal.AddCommand({ { "adc", "a", "Configure adc. Ex: adc 1000" },
            [this](const infra::BoundedConstString& param)
            {
                this->terminal.ProcessResult(ConfigureAdc(param));
            } });
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::ConfigurePwm(const infra::BoundedConstString& param)
    {
        infra::Tokenizer tokenizer(param, ' ');

        if (tokenizer.Size() != 2)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments" };

        auto deadTime = ParseInput(tokenizer.Token(0));
        if (!deadTime)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float between 500 and 2000." };

        auto frequency = ParseInput(tokenizer.Token(1));
        if (!frequency)
            return { services::TerminalWithStorage::Status::error, "invalid value. It should be a float between 10000 and 20000." };

        return { services::TerminalWithStorage::Status::success };
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::ConfigureAdc(const infra::BoundedConstString& param)
    {
        // Sample and hold
        return { services::TerminalWithStorage::Status::success };
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::Start()
    {
        return { services::TerminalWithStorage::Status::success };
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::Stop()
    {
        return { services::TerminalWithStorage::Status::success };
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::ReadAdcWithSampleTime()
    {
        return { services::TerminalWithStorage::Status::success };
    }

    TerminalInteractor::StatusWithMessage TerminalInteractor::SetPwmDuty(const infra::BoundedConstString& param)
    {
        infra::Tokenizer tokenizer(param, ' ');

        if (tokenizer.Size() != 3)
            return { services::TerminalWithStorage::Status::error, "invalid number of arguments" };

        auto dutyA = ParseInput(tokenizer.Token(0));
        if (!dutyA)
            return { services::TerminalWithStorage::Status::error, "invalid value for phase A. It should be a float between 0 and 100." };

        auto dutyB = ParseInput(tokenizer.Token(1));
        if (!dutyB)
            return { services::TerminalWithStorage::Status::error, "invalid value for phase B. It should be a float between 0 and 100." };

        auto dutyC = ParseInput(tokenizer.Token(2));
        if (!dutyC)
            return { services::TerminalWithStorage::Status::error, "invalid value for phase C. It should be a float between 0 and 100." };

        return { services::TerminalWithStorage::Status::success };
    }
}
