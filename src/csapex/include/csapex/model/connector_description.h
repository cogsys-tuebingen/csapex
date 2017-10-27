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
    AUUID owner;
    ConnectorType connector_type;
    std::string label;
    bool optional;
    bool is_parameter;

    TokenDataConstPtr token_type;

    UUID id;

    // connected connectors (UUID, is_active)
    struct Target : public Serializable
    {
        AUUID auuid;
        bool active;

        Target()
            : auuid(UUID::NONE), active(false)
        {}
        Target(AUUID id, bool active)
            : auuid(id), active(active)
        {}

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(const SerializationBuffer& data) override;

    protected:
        virtual std::shared_ptr<Clonable> makeEmptyClone() const override;
    };

    std::vector<Target> targets;

    bool valid;

    ConnectorDescription();

    ConnectorDescription(const AUUID& owner,
                         ConnectorType connector_type,
                         const TokenDataConstPtr& token_type,
                         const std::string& label,
                         bool optional = true,
                         bool is_parameter = false);

    ConnectorDescription(const AUUID& owner,
                         ConnectorType connector_type,
                         const std::string& label,
                         bool optional = true,
                         bool is_parameter = false);

    ConnectorDescription(const AUUID& owner,
                         const UUID& uuid,
                         ConnectorType connector_type,
                         const TokenDataConstPtr& token_type,
                         const std::string& label,
                         bool optional = true,
                         bool is_parameter = false);

    bool isOutput() const;

    AUUID getAUUID() const;

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(const SerializationBuffer& data) override;

protected:
    virtual std::shared_ptr<Clonable> makeEmptyClone() const override;
};

}

#endif // CONNECTOR_DESCRIPTION_H
