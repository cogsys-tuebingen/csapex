#ifndef NODE_SERIALIZER_H
#define NODE_SERIALIZER_H

/// COMPONENT
#include <csapex/utility/singleton.hpp>
#include <csapex/model/model_fwd.h>
#include <csapex_core/csapex_core_export.h>

/// SYSTEM
#include <map>
#include <functional>
#include <typeindex>

#define CSAPEX_REGISTER_SERIALIZER_NS(Node, Serializer,instancename) \
namespace csapex { \
namespace serializion { \
    static SerializationRegistered<Node, Serializer> instancename; \
} \
}
#define CSAPEX_REGISTER_SERIALIZER(Node, Serializer)\
    CSAPEX_REGISTER_SERIALIZER_NS(Node, Serializer,g_instance_)

namespace YAML
{
class Node;
}

namespace csapex
{
class CSAPEX_CORE_EXPORT NodeSerializer : public Singleton<NodeSerializer>
{
public:
    template <typename N, typename Instance>
    static void registerInstance()
    {
        instance().serializers[std::type_index(typeid(const N))] = [](const csapex::Node& node, YAML::Node& doc) {
            Instance::serialize(static_cast<const N&>(node), doc);
        };
        instance().deserializers[std::type_index(typeid(const N))] = [](csapex::Node& node, const YAML::Node& doc) {
            Instance::deserialize(static_cast<N&>(node), doc);
        };
    }

    void shutdown() override;

    void serialize(const csapex::Node& node, YAML::Node& doc);
    void deserialize(csapex::Node& node, const YAML::Node& doc);

private:
    std::map<std::type_index, std::function<void(const csapex::Node&, YAML::Node&)>> serializers;
    std::map<std::type_index, std::function<void(csapex::Node&, const YAML::Node&)>> deserializers;
};
}


namespace csapex {
namespace serializion {

template <typename N, typename Instance>
struct SerializationRegistered
{
    SerializationRegistered()
    {
        csapex::NodeSerializer::template registerInstance<N, Instance>();
    }
};

}
}

#endif // NODE_SERIALIZER_H

