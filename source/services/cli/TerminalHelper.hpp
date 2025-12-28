#include "infra/stream/StringInputStream.hpp"
#include <optional>

namespace services
{
    inline std::optional<float> ParseInput(const infra::BoundedConstString& data)
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
