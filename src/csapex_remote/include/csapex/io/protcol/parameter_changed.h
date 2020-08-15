#ifndef PARAMETER_CHANGED_H
#define PARAMETER_CHANGED_H

/// PROJECT
#include <csapex/io/broadcast_impl.hpp>
#include <csapex/model/notification.h>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/any.h>

namespace csapex
{
namespace io
{
class ParameterChangedSerializer;
}

class ParameterChanged : public BroadcastImplementation<ParameterChanged>
{
public:
    ParameterChanged(const UUID& id, const std::any& value);
    ParameterChanged();

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    AUUID getUUID() const;
    std::any getValue() const;

private:
    AUUID uuid;

    std::any value;
};

}  // namespace csapex

#endif  // PARAMETER_CHANGED_H
