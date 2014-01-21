#ifndef VECTOR_MESSAGE_H
#define VECTOR_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <string>

namespace csapex {
namespace connection_types {

struct VectorMessage : public Message
{
    typedef boost::shared_ptr<VectorMessage> Ptr;

    VectorMessage();

    ConnectionType::Ptr getSubType() const;

    template <typename T>
    static VectorMessage::Ptr make()
    {
        return VectorMessage::Ptr (new VectorMessage(T::make()));
    }

    static VectorMessage::Ptr make();

    virtual ConnectionType::Ptr clone();
    virtual ConnectionType::Ptr toType();

    virtual bool canConnectTo(const ConnectionType* other_side) const;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const;

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);

private:
    VectorMessage(ConnectionType::Ptr type);

public:
    std::vector<ConnectionType::Ptr> value;

private:
    ConnectionType::Ptr type_;

};

}
}

#endif // VECTOR_MESSAGE_H
