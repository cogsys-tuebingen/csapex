#ifndef MOCKUP_MSGS_H
#define MOCKUP_MSGS_H

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <string>

namespace csapex
{
class Mock
{
public:
    std::string payload;
};

struct Foo
{
    Foo() : value(-1)
    {
    }
    Foo(int v) : value(v)
    {
    }

    int value;
};

struct Base
{
};
struct Child : public Base
{
};

SerializationBuffer& operator<<(SerializationBuffer& data, const Mock& t);
const SerializationBuffer& operator>>(const SerializationBuffer& data, Mock& t);

SerializationBuffer& operator<<(SerializationBuffer& data, const Foo& t);
const SerializationBuffer& operator>>(const SerializationBuffer& data, Foo& t);

SerializationBuffer& operator<<(SerializationBuffer& data, const Base& t);
const SerializationBuffer& operator>>(const SerializationBuffer& data, Base& t);

SerializationBuffer& operator<<(SerializationBuffer& data, const Child& t);
const SerializationBuffer& operator>>(const SerializationBuffer& data, Child& t);

namespace connection_types
{
class MockMessage : public MessageTemplate<Mock, MockMessage>
{
};

template <>
struct type<MockMessage>
{
    static std::string name()
    {
        return "MockMessage";
    }
};

struct VectorMessage : public MessageTemplate<std::vector<Foo>, VectorMessage>
{
};

template <>
struct type<VectorMessage>
{
    static std::string name()
    {
        return "TestVector";
    }
};

struct BaseMessage : public MessageTemplate<Base, BaseMessage>
{
};

template <>
struct type<BaseMessage>
{
    static std::string name()
    {
        return "Base";
    }
};

struct ChildMessage : public MessageTemplate<Child, ChildMessage>
{
public:
    int child_value;
};

template <>
struct type<ChildMessage>
{
    static std::string name()
    {
        return "Child";
    }
};

}  // namespace connection_types
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct convert<csapex::connection_types::MockMessage>
{
    static Node encode(const csapex::connection_types::MockMessage& rhs)
    {
        Node n;
        n["payload"] = rhs.value.payload;
        return n;
    }

    static bool decode(const Node& node, csapex::connection_types::MockMessage& rhs)
    {
        if (node["payload"].IsDefined()) {
            rhs.value.payload = node["payload"].as<std::string>();
        } else {
            return false;
        }
        return true;
    }
};

template <>
struct CSAPEX_CORE_EXPORT convert<csapex::Foo>
{
    static Node encode(const csapex::Foo& rhs)
    {
        Node node;
        node["value"] = rhs.value;
        return node;
    }
    static bool decode(const Node& node, csapex::Foo& rhs)
    {
        rhs.value = node["value"].as<int>();
        return true;
    }
};

template <>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::VectorMessage>
{
    static Node encode(const csapex::connection_types::VectorMessage& rhs)
    {
        Node node;
        node["value"] = rhs.value;
        return node;
    }
    static bool decode(const Node& node, csapex::connection_types::VectorMessage& rhs)
    {
        rhs.value = node["value"].as<std::vector<csapex::Foo>>();
        return true;
    }
};

template <>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::BaseMessage>
{
    static Node encode(const csapex::connection_types::BaseMessage& rhs)
    {
        return {};
    }
    static bool decode(const Node& node, csapex::connection_types::BaseMessage& rhs)
    {
        return true;
    }
};
template <>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::ChildMessage>
{
    static Node encode(const csapex::connection_types::ChildMessage& rhs)
    {
        return {};
    }
    static bool decode(const Node& node, csapex::connection_types::ChildMessage& rhs)
    {
        return true;
    }
};
}  // namespace YAML

#endif  // MOCKUP_MSGS_H
