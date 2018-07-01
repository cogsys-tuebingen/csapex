/// HEADER
#include <csapex/serialization/node_serializer.h>

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

void NodeSerializer::serialize(const csapex::Node& node, YAML::Node& doc)
{
    auto fn = serializers.find(std::type_index(typeid(node)));
    if (fn != serializers.end()) {
        (fn->second)(node, doc);
    }
}

void NodeSerializer::deserialize(csapex::Node& node, const YAML::Node& doc)
{
    const csapex::Node& const_node = node;
    auto fn = deserializers.find(std::type_index(typeid(const_node)));
    if (fn != deserializers.end()) {
        (fn->second)(node, doc);
    }
}

void NodeSerializer::shutdown()
{
    serializers.clear();
    deserializers.clear();
}
