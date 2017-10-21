#ifndef CONNECTION_INFORMATION_H
#define CONNECTION_INFORMATION_H

/// COMPONENT
#include <csapex/serialization/serializable.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/model_fwd.h>
#include <csapex/csapex_export.h>
#include <csapex/model/fulcrum.h>

namespace csapex
{

struct CSAPEX_EXPORT ConnectionInformation : public Serializable
{
    UUID from;
    UUID to;
    std::string from_label;
    std::string to_label;
    TokenDataConstPtr type;

    int id;

    bool active;

    std::vector<Fulcrum> fulcrums;

    ConnectionInformation(const UUID& from, const UUID& to,
                          const TokenDataConstPtr &type, int id,
                          bool active, const std::vector<Fulcrum>& fulcrums);

    ConnectionInformation(const ConnectionInformation& other);

    ConnectionInformation& operator = (const ConnectionInformation& other);

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(const SerializationBuffer& data) override;

    bool operator == (const ConnectionInformation& other) const;

protected:
    virtual std::shared_ptr<Clonable> makeEmptyClone() const override;

private:
    friend class SerializationBuffer;
    ConnectionInformation();
};

}

#endif // CONNECTION_INFORMATION_H
