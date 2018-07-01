#ifndef CONNECTION_DESCRIPTION_H
#define CONNECTION_DESCRIPTION_H

/// COMPONENT
#include <csapex/serialization/serializable.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/model_fwd.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/model/fulcrum.h>

namespace csapex
{
struct CSAPEX_CORE_EXPORT ConnectionDescription : public Serializable
{
protected:
    CLONABLE_IMPLEMENTATION(ConnectionDescription);

public:
    UUID from;
    UUID to;
    std::string from_label;
    std::string to_label;
    TokenDataConstPtr type;

    int id;

    bool active;

    std::vector<Fulcrum> fulcrums;

    ConnectionDescription(const UUID& from, const UUID& to, const TokenDataConstPtr& type, int id, bool active, const std::vector<Fulcrum>& fulcrums);

    ConnectionDescription(const ConnectionDescription& other);

    ConnectionDescription& operator=(const ConnectionDescription& other);

    ConnectionDescription();

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    bool operator==(const ConnectionDescription& other) const;
};

}  // namespace csapex

#endif  // CONNECTION_DESCRIPTION_H
