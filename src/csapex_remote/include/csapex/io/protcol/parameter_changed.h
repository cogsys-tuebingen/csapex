#ifndef PARAMETER_CHANGED_H
#define PARAMETER_CHANGED_H

/// PROJECT
#include <csapex/io/broadcast_impl.hpp>
#include <csapex/model/notification.h>
#include <csapex/serialization/serialization_fwd.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{
namespace io
{
class ParameterChangedSerializer;
}

class ParameterChanged : public BroadcastImplementation<ParameterChanged>
{
public:
    ParameterChanged(const UUID& id, const boost::any& value);
    ParameterChanged();

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    AUUID getUUID() const;
    boost::any getValue() const;

private:
    AUUID uuid;

    boost::any value;
};

}  // namespace csapex

#endif  // PARAMETER_CHANGED_H
