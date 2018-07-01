#ifndef CONNECTOR_DESCRIPTION_H
#define CONNECTOR_DESCRIPTION_H

/// PROJECT
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/model_fwd.h>
#include <csapex/serialization/serializable.h>

namespace csapex
{
struct ConnectorDescription : public Serializable
{
protected:
    CLONABLE_IMPLEMENTATION(ConnectorDescription);

public:
    AUUID owner;
    ConnectorType connector_type;
    std::string label;
    bool optional;
    bool is_parameter;
    bool is_variadic;

    TokenDataConstPtr token_type;

    UUID id;

    // connected connectors (UUID, is_active)
    struct Target : public Serializable
    {
    protected:
        CLONABLE_IMPLEMENTATION(Target);

    public:
        AUUID auuid;
        bool active;

        Target() : auuid(UUID::NONE), active(false)
        {
        }
        Target(AUUID id, bool active) : auuid(id), active(active)
        {
        }

        virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;
    };

    std::vector<Target> targets;

    bool valid;

    ConnectorDescription();

    ConnectorDescription(const AUUID& owner, ConnectorType connector_type, const TokenDataConstPtr& token_type, const std::string& label);

    ConnectorDescription(const AUUID& owner, ConnectorType connector_type, const std::string& label);

    ConnectorDescription(const AUUID& owner, const UUID& uuid, ConnectorType connector_type, const TokenDataConstPtr& token_type, const std::string& label);

    ConnectorDescription& setOptional(bool optional);
    ConnectorDescription& setParameter(bool parameter);
    ConnectorDescription& setVariadic(bool variadic);

    bool isOutput() const;

    AUUID getAUUID() const;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;
};

}  // namespace csapex

#endif  // CONNECTOR_DESCRIPTION_H
