#include "infra/stream/StringInputStream.hpp"
#include <optional>

namespace services
{
    inline std::optional<float> ParseInput(const infra::BoundedConstString& data, float minValue = -std::numeric_limits<float>::infinity(), float maxValue = std::numeric_limits<float>::infinity())
    {
        float value = 0.0f;
        infra::StringInputStream stream(data, infra::softFail);
        stream >> value;

        if (!stream.ErrorPolicy().Failed() && value >= minValue && value <= maxValue)
            return std::make_optional(value);
        else
            return {};
    }
}
