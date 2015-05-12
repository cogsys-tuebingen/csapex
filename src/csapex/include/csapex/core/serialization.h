#ifndef SERIALIZATION_H
#define SERIALIZATION_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <map>
#include <functional>


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
class Serialization : public Singleton<Serialization>
{
public:
    template <typename N, typename Instance>
    static void registerInstance()
    {
        instance().serializers[&typeid(const N)] = [](const csapex::Node& node, YAML::Node& doc) {
            Instance::serialize(static_cast<const N&>(node), doc);
        };
        instance().deserializers[&typeid(const N)] = [](csapex::Node& node, const YAML::Node& doc) {
            Instance::deserialize(static_cast<N&>(node), doc);
        };
    }

    void serialize(const csapex::Node& node, YAML::Node& doc);
    void deserialize(csapex::Node& node, const YAML::Node& doc);

private:
    std::map<const std::type_info*, std::function<void(const csapex::Node&, YAML::Node&)>> serializers;
    std::map<const std::type_info*, std::function<void(csapex::Node&, const YAML::Node&)>> deserializers;
};
}


namespace csapex {
namespace serializion {

template <typename N, typename Instance>
struct SerializationRegistered
{
    SerializationRegistered()
    {
        csapex::Serialization::template registerInstance<N, Instance>();
    }
};

}
}

#endif // SERIALIZATION_H

