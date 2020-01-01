#ifndef GENERIC_VECTOR_MESSAGE_HPP
#define GENERIC_VECTOR_MESSAGE_HPP

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/msg/token_traits.h>
#include <csapex/msg/any_message.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/serialization/yaml.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <string>
#include <boost/static_assert.hpp>
#include <vector>
#undef NDEBUG
#include <assert.h>

namespace YAML
{
template <typename T, typename S>
struct as_if;
}

namespace csapex
{
namespace connection_types
{
template <typename Type>
struct GenericPointerMessage;
template <typename Type>
struct GenericValueMessage;

class CSAPEX_CORE_EXPORT GenericVectorMessage : public Message
{
protected:
    CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(GenericVectorMessage);

    friend class YAML::as_if<GenericVectorMessage, void>;

public:
    struct Anything
    {
    };

private:
    struct CSAPEX_CORE_EXPORT EntryInterface : public Message
    {
        typedef std::shared_ptr<EntryInterface> Ptr;

        EntryInterface(const std::string& name, Message::Stamp stamp = 0) : Message(name, "/", stamp)
        {
        }

        virtual std::string nestedName() const = 0;

        virtual void encode(YAML::Node& node) const = 0;
        virtual void decode(const YAML::Node& node) = 0;
    };

    template <typename T>
    struct Implementation : public EntryInterface
    {
    protected:
        CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(Implementation<T>);

    private:
        template <typename Type, typename Enable = void>
        struct ExtractPayload
        {
            typedef Type raw_type;
            typedef Type type;
        };
        template <typename Type>
        struct ExtractPayload<std::shared_ptr<Type>, void>
        {
            typedef Type raw_type;
            typedef std::shared_ptr<Type const> type;
        };

        typedef typename ExtractPayload<T>::type Payload;

        typedef Implementation<T> Self;

    public:
        typedef std::shared_ptr<Self> Ptr;

    public:
        Implementation() : EntryInterface(std::string("std::vector<") + type2nameWithoutNamespace(typeid(T)) + ">")
        {
            static_assert(!std::is_same<T, void*>::value, "void* not allowed");
            value.reset(new std::vector<Payload>);
        }

        static typename Self::Ptr make()
        {
            return Self::Ptr(new Self);
        }

        bool cloneData(const Implementation<T>& other)
        {
            value.reset(new std::vector<Payload>);
            *value = *other.value;
            return true;
        }

        virtual bool canConnectTo(const TokenData* other_side) const override
        {
            if (const EntryInterface* ei = dynamic_cast<const EntryInterface*>(other_side)) {
                return nestedType()->canConnectTo(ei->nestedType().get());
            } else {
                const GenericVectorMessage* vec = dynamic_cast<const GenericVectorMessage*>(other_side);
                if (vec != 0) {
                    // if the other side is a vector too, try if they are compatible
                    return vec->canConnectTo(this);
                } else {
                    // the other side is not a vector
                    // try if the nested type is compatible
                    auto type = nestedType();
                    bool nested_is_compatible = other_side->canConnectTo(type.get());
                    if (nested_is_compatible) {
                        // connectable via iteration
                        return true;
                    } else {
                        // no iteration possible, default to asking
                        return other_side->acceptsConnectionFrom(this);
                    }
                    // return dynamic_cast<const AnyMessage*> (other_side) != nullptr;
                }
            }
        }
        virtual bool acceptsConnectionFrom(const TokenData* other_side) const override
        {
            if (const EntryInterface* ei = dynamic_cast<const EntryInterface*>(other_side)) {
                return nestedType()->canConnectTo(ei->nestedType().get());
            } else {
                return false;
            }
        }

        std::string nestedName() const override
        {
            return type2name(typeid(T));
        }

        void encode(YAML::Node& node) const override
        {
            node["values"] = *value;
        }

        void decode(const YAML::Node& node) override
        {
            YAML::Emitter emitter;
            emitter << node;
            value.reset(new std::vector<Payload>);
            *value = node["values"].as<std::vector<Payload>>();
        }

        TokenData::Ptr nestedType() const override
        {
            return makeTypeSwitch(Tag<Payload>());
        }

        virtual void addNestedValue(const TokenData::ConstPtr& msg) override
        {
            addCastedEntry(*value, msg);
        }
        virtual TokenData::ConstPtr nestedValue(std::size_t i) const override
        {
            return makeToken(value->at(i));
        }
        virtual std::size_t nestedValueCount() const override
        {
            return value->size();
        }

        template <typename MsgType>
        void addCastedEntry(std::vector<std::shared_ptr<MsgType>>&, const TokenData::ConstPtr& ptr, typename std::enable_if<std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            // internal: vector of msg pointers
            auto casted = std::dynamic_pointer_cast<MsgType const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(casted);
        }
        template <typename MsgType>
        void addCastedEntry(std::vector<MsgType>&, const TokenData::ConstPtr& ptr, typename std::enable_if<std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            // internal: vector of msg values
            auto casted = std::dynamic_pointer_cast<MsgType const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(*casted);
        }

        //        template <typename MsgType>
        //        void addCastedEntry(std::vector<std::shared_ptr<MsgType>>& ,
        //                            const TokenData::ConstPtr& ptr,
        //                            typename std::enable_if<!std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        //        {
        //            // internal: vector of non-msg pointers
        //            auto casted = std::dynamic_pointer_cast<MsgType const>(ptr);
        //        }
        template <typename MsgType>
        void addCastedEntry(std::vector<std::shared_ptr<MsgType>>&, const TokenData::ConstPtr& ptr,
                            typename std::enable_if<!std::is_base_of<TokenData, MsgType>::value && connection_types::should_use_pointer_message<MsgType>::value>::type* = 0)
        {
            // internal: vector of non-msg values
            auto casted = std::dynamic_pointer_cast<connection_types::GenericPointerMessage<MsgType> const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(casted->value);
        }
        template <typename MsgType>
        void addCastedEntry(std::vector<MsgType>&, const TokenData::ConstPtr& ptr,
                            typename std::enable_if<!std::is_base_of<TokenData, MsgType>::value && connection_types::should_use_pointer_message<MsgType>::value>::type* = 0)
        {
            // internal: vector of non-msg values
            auto casted = std::dynamic_pointer_cast<connection_types::GenericPointerMessage<MsgType> const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(*casted->value);
        }
        template <typename ValueType>
        void addCastedEntry(std::vector<ValueType>&, const TokenData::ConstPtr& ptr,
                            typename std::enable_if<!std::is_base_of<TokenData, ValueType>::value && connection_types::should_use_value_message<ValueType>::value>::type* = 0)
        {
            // internal: vector of non-msg values
            auto casted = std::dynamic_pointer_cast<connection_types::GenericValueMessage<ValueType> const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(casted->value);
        }

        template <typename MsgType>
        static TokenData::ConstPtr makeToken(const std::shared_ptr<MsgType>& ptr, typename std::enable_if<std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            return ptr;
        }

        template <typename MsgType>
        static TokenData::ConstPtr makeToken(const MsgType& val, typename std::enable_if<std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            return std::make_shared<MsgType>(val);
        }
        template <typename MsgType>
        static TokenData::ConstPtr makeToken(const MsgType& val, typename std::enable_if<!std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            return convertToken(val);
        }
        // ros message by value
        template <typename MsgType>
        static TokenData::ConstPtr convertToken(const MsgType& val, typename std::enable_if<connection_types::should_use_pointer_message<MsgType>::value>::type* = 0)
        {
            auto res = csapex::makeEmpty<connection_types::GenericPointerMessage<MsgType>>();
            res->value = std::make_shared<MsgType>(val);
            return res;
        }
        // ros message by pointer
        template <typename MsgType>
        static TokenData::ConstPtr convertToken(const std::shared_ptr<MsgType>& val, typename std::enable_if<connection_types::should_use_pointer_message<MsgType>::value>::type* = 0)
        {
            auto res = csapex::makeEmpty<connection_types::GenericPointerMessage<MsgType>>();
            res->value = val;
            return res;
        }
        // value message
        template <typename MsgType>
        static TokenData::ConstPtr convertToken(const MsgType& val, typename std::enable_if<connection_types::should_use_value_message<MsgType>::value>::type* = 0)
        {
            auto res = csapex::makeEmpty<connection_types::GenericValueMessage<MsgType>>();
            res->value = val;
            return res;
        }

        template <class Type>
        struct Tag
        {
        };

        template <typename MsgType>
        static TokenData::Ptr makeTypeSwitch(const Tag<std::shared_ptr<MsgType>>& ptr)
        {
            return makeTypeImpl<MsgType>();
        }
        template <typename MsgType>
        static TokenData::Ptr makeTypeSwitch(const Tag<MsgType>& type)
        {
            return makeTypeImpl<MsgType>();
        }

        template <typename MsgType>
        static TokenData::Ptr makeTypeImpl(typename std::enable_if<connection_types::should_use_pointer_message<MsgType>::value>::type* = 0)
        {
            return csapex::makeEmpty<connection_types::GenericPointerMessage<MsgType>>();
        }

        template <typename MsgType>
        static TokenData::Ptr makeTypeImpl(typename std::enable_if<connection_types::should_use_value_message<MsgType>::value>::type* = 0)
        {
            return csapex::makeEmpty<connection_types::GenericValueMessage<MsgType>>();
        }

        template <typename MsgType>
        static TokenData::Ptr
        makeTypeImpl(typename std::enable_if<!connection_types::should_use_pointer_message<MsgType>::value && !connection_types::should_use_value_message<MsgType>::value>::type* = 0)
        {
            static_assert(std::is_base_of<TokenData, MsgType>::value, "message has to be derived from TokenData");
            return csapex::makeEmpty<typename std::remove_const<MsgType>::type>();
        }

    public:
        std::shared_ptr<std::vector<Payload>> value;
    };

    template <typename T>
    struct MessageImplementation : public Implementation<T>
    {
    protected:
        CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(MessageImplementation<T>);

    public:
        typedef Implementation<T> Parent;
        typedef MessageImplementation<T> Self;

        using Parent::value;

    public:
        typedef std::shared_ptr<Self> Ptr;

        static typename Self::Ptr make()
        {
            return Self::Ptr(new Self);
        }
        bool cloneData(const MessageImplementation<T>& other)
        {
            return Parent::cloneData(other);
        }

        std::string nestedName() const override
        {
            return type2name(typeid(T));
        }

        void encode(YAML::Node& node) const override
        {
            node["values"] = YAML::Node(YAML::NodeType::Sequence);
            for (const auto& entry : *value) {
                node["values"].push_back(MessageSerializer::serializeYamlMessage(entry));
            }
        }
        void decode(const YAML::Node& node) override
        {
            for (const YAML::Node& centry : node["values"]) {
                std::shared_ptr<T> msg;
                if (!centry["type"].IsDefined()) {
                    // assume legacy messages
                    YAML::Node entry = centry;
                    entry["type"] = node["value_type"];
                    entry["data"] = centry;
                    msg = std::dynamic_pointer_cast<T>(MessageSerializer::deserializeYamlMessage(entry));
                } else {
                    msg = std::dynamic_pointer_cast<T>(MessageSerializer::deserializeYamlMessage(centry));
                }
                value->push_back(*msg);
            }
        }

        void serialize(SerializationBuffer& data, SemanticVersion& version) const override
        {
            TokenData::serialize(data, version);
            data << value;
        }
        void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override
        {
            TokenData::deserialize(data, version);
            data >> value;
        }
    };

    struct CSAPEX_CORE_EXPORT AnythingImplementation : public EntryInterface
    {
    protected:
        CLONABLE_IMPLEMENTATION(AnythingImplementation);

    public:
        AnythingImplementation();

        bool canConnectTo(const TokenData* other_side) const override;
        bool acceptsConnectionFrom(const TokenData* other_side) const override;
        void encode(YAML::Node& node) const override;
        void decode(const YAML::Node& node) override;

        void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        std::string nestedName() const override
        {
            return "::anything::";
        }
    };

    struct CSAPEX_CORE_EXPORT InstancedImplementation : public EntryInterface
    {
    protected:
        CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(InstancedImplementation);

        friend class GenericVectorMessage;

    public:
        InstancedImplementation(TokenData::ConstPtr type);

        bool canConnectTo(const TokenData* other_side) const override;
        bool acceptsConnectionFrom(const TokenData* other_side) const override;
        void encode(YAML::Node& node) const override;
        void decode(const YAML::Node& node) override;

        TokenData::Ptr nestedType() const override;

        void addNestedValue(const TokenData::ConstPtr& msg) override;
        TokenData::ConstPtr nestedValue(std::size_t i) const override;
        std::size_t nestedValueCount() const override;

        void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        std::string nestedName() const override
        {
            return "::instanced::";
        }

        bool cloneData(const InstancedImplementation& other);

    private:
        InstancedImplementation();

    private:
        TokenData::ConstPtr type_;
        std::vector<TokenDataPtr> value;
    };

public:
    typedef std::shared_ptr<GenericVectorMessage> Ptr;
    typedef std::shared_ptr<const GenericVectorMessage> ConstPtr;

    bool cloneData(const GenericVectorMessage& other)
    {
        pimpl = other.pimpl->cloneAs<EntryInterface>();
        return true;
    }

    template <typename T>
    struct TypeMap
    {
        typedef std::vector<T> type;
        typedef std::shared_ptr<type> Ptr;
        typedef std::shared_ptr<type const> ConstPtr;
    };

    template <typename T, class Enable = void>
    struct Adder
    {
        static void addToMap(std::map<std::string, EntryInterface::Ptr>& map, const std::string& type_name)
        {
            map[type_name].reset(new InstancedImplementation(T()));
        }
    };

    template <typename T>
    struct Adder<T, typename std::enable_if<std::is_base_of<TokenData, T>::value>::type>
    {
        static void addToMap(std::map<std::string, EntryInterface::Ptr>& map, const std::string& type_name)
        {
            map[type_name].reset(new MessageImplementation<T>());
        }
    };

    template <typename T>
    struct Adder<T, typename std::enable_if<!std::is_base_of<TokenData, T>::value && !std::is_same<T, Anything>::value>::type>
    {
        static void addToMap(std::map<std::string, EntryInterface::Ptr>& map, const std::string& type_name)
        {
            map[type_name].reset(new Implementation<T>());
        }
    };

    template <typename T>
    struct Adder<T, typename std::enable_if<std::is_same<Anything, T>::value>::type>
    {
        static void addToMap(std::map<std::string, EntryInterface::Ptr>& map, const std::string& type_name)
        {
            map[type_name].reset(new AnythingImplementation());
        }
    };

    struct SupportedTypes : public Singleton<SupportedTypes>
    {
        static EntryInterface::Ptr make(const std::string& type)
        {
            if (type == "::anything::") {
                return std::make_shared<AnythingImplementation>();
            }

            const std::string ns = "csapex::connection_types::";
            std::string ns_type = type;
            if (type.find(ns) == std::string::npos) {
                ns_type = ns + type;
            }
            auto pos = instance().map_.find(ns_type);
            if (pos == instance().map_.end()) {
                throw std::runtime_error(std::string("cannot make vector of type ") + type);
            }
            return std::dynamic_pointer_cast<EntryInterface>(pos->second->makeEmptyInstance());
        }

        template <typename T>
        static void registerType(const std::string& type_name = "")
        {
            std::string type = type_name.empty() ? type2name(typeid(T)) : type_name;
            std::map<std::string, EntryInterface::Ptr>& map = instance().map_;
            if (map.find(type) == map.end()) {
                Adder<T>::addToMap(instance().map_, type);
            }
        }

        template <typename T>
        static void deregisterType(const std::string& type_name = "")
        {
            std::string type = type_name.empty() ? type2name(typeid(T)) : type_name;
            std::map<std::string, EntryInterface::Ptr>& map = instance().map_;
            auto pos = map.find(type);
            if (pos != map.end()) {
                map.erase(pos);
            }
        }

        void shutdown() override
        {
            map_.clear();
        }

    private:
        std::map<std::string, EntryInterface::Ptr> map_;
    };

    template <typename T>
    static void registerType(const std::string& type_name = "")
    {
        SupportedTypes::registerType<T>(type_name);
    }

    template <typename T>
    static void deregisterType(const std::string& type_name = "")
    {
        // TODO: support deregistering
        // For that, this singleton must be initialized somewhere else...
        // Alternative: Get rid of the singleton!
        // SupportedTypes::deregisterType<T>(type_name);
    }

    template <typename T>
    static GenericVectorMessage::Ptr make(typename std::enable_if<std::is_base_of<TokenData, T>::value>::type* dummy = 0)
    {
        registerType<T>();
        return GenericVectorMessage::Ptr(new GenericVectorMessage(MessageImplementation<T>::make(), "/", 0));
    }

    template <typename T>
    static GenericVectorMessage::Ptr make(typename std::enable_if<!std::is_base_of<TokenData, T>::value && !std::is_same<T, Anything>::value>::type* dummy = 0)
    {
        registerType<T>();
        return GenericVectorMessage::Ptr(new GenericVectorMessage(Implementation<T>::make(), "/", 0));
    }

    template <typename T>
    static GenericVectorMessage::Ptr make(typename std::enable_if<std::is_same<Anything, T>::value>::type* dummy = 0)
    {
        return GenericVectorMessage::Ptr(new GenericVectorMessage(std::make_shared<AnythingImplementation>(), "/", 0));
    }

    static GenericVectorMessage::Ptr make(TokenData::ConstPtr type)
    {
        return GenericVectorMessage::Ptr(new GenericVectorMessage(std::make_shared<InstancedImplementation>(type), "/", 0));
    }

    template <typename T>
    std::shared_ptr<std::vector<T> const> makeShared(typename std::enable_if<std::is_base_of<TokenData, T>::value>::type* = 0) const
    {
        if (auto impl = std::dynamic_pointer_cast<Implementation<T>>(pimpl)) {
            return impl->value;
        } else if (auto instance = std::dynamic_pointer_cast<InstancedImplementation>(pimpl)) {
            auto res = std::make_shared<std::vector<T>>();
            for (const TokenDataConstPtr& td : instance->value) {
                if (auto v = std::dynamic_pointer_cast<T const>(td)) {
                    res->push_back(*v);
                }
            }
            return res;
        } else {
            throw std::runtime_error("cannot make the msg vector shared");
        }
    }
    template <typename T>
    std::shared_ptr<std::vector<T> const> makeShared(typename std::enable_if<!std::is_base_of<TokenData, T>::value && !should_use_value_message<T>::value>::type* = 0) const
    {
        if (auto impl = std::dynamic_pointer_cast<Implementation<T>>(pimpl)) {
            return impl->value;
        } else if (auto instance = std::dynamic_pointer_cast<InstancedImplementation>(pimpl)) {
            auto res = std::make_shared<std::vector<T>>();
            makeSharedValue(instance.get(), res);
            return res;
        } else {
            throw std::runtime_error("cannot make the direct vector shared");
        }
    }
    template <typename T>
    std::shared_ptr<std::vector<T> const> makeShared(typename std::enable_if<!std::is_base_of<TokenData, T>::value && should_use_value_message<T>::value>::type* = 0) const
    {
        if (auto message = std::dynamic_pointer_cast<MessageImplementation<GenericValueMessage<T>>>(pimpl)) {
            auto res = std::make_shared<std::vector<T>>();
            for (auto entry : *message->value) {
                res->push_back(entry.value);
            }
            return res;
        } else if (auto impl = std::dynamic_pointer_cast<Implementation<T>>(pimpl)) {
            return impl->value;
        } else if (auto instance = std::dynamic_pointer_cast<InstancedImplementation>(pimpl)) {
            auto res = std::make_shared<std::vector<T>>();
            makeSharedValue(instance.get(), res);
            return res;
        } else {
            throw std::runtime_error("cannot make the direct vector shared");
        }
    }

    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<T>>& res, typename std::enable_if<std::is_base_of<TokenData, T>::value>::type* = 0)
    {
        for (const TokenDataConstPtr& td : i->value) {
            res->push_back(*std::dynamic_pointer_cast<T const>(td));
        }
    }
    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<std::shared_ptr<T>>>& res, typename std::enable_if<std::is_base_of<TokenData, T>::value>::type* = 0)
    {
        for (const TokenDataConstPtr& td : i->value) {
            res->push_back(std::dynamic_pointer_cast<T>(td));
        }
    }
    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<std::shared_ptr<T const>>>& res, typename std::enable_if<std::is_base_of<TokenData, T>::value>::type* = 0)
    {
        for (const TokenDataConstPtr& td : i->value) {
            res->push_back(std::dynamic_pointer_cast<T const>(td));
        }
    }

    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<T>>& res, typename std::enable_if<connection_types::should_use_pointer_message<T>::value>::type* = 0)
    {
        static_assert(!std::is_base_of<TokenData, T>::value, "not applicable to messages");
        for (const TokenDataConstPtr& td : i->value) {
            res->push_back(*std::dynamic_pointer_cast<GenericPointerMessage<T> const>(td)->value);
        }
    }
    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<std::shared_ptr<T>>>& res,
                                typename std::enable_if<connection_types::should_use_pointer_message<T>::value>::type* = 0)
    {
        static_assert(!std::is_base_of<TokenData, T>::value, "not applicable to messages");
        for (const TokenDataConstPtr& td : i->value) {
            res->push_back(std::dynamic_pointer_cast<GenericPointerMessage<T> const>(td)->value);
        }
    }
    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<std::shared_ptr<T const>>>& res,
                                typename std::enable_if<connection_types::should_use_pointer_message<T>::value>::type* = 0)
    {
        static_assert(!std::is_base_of<TokenData, T>::value, "not applicable to messages");
        for (const TokenDataConstPtr& td : i->value) {
            res->push_back(std::dynamic_pointer_cast<GenericPointerMessage<T const> const>(td)->value);
        }
    }

    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<T>>& res, typename std::enable_if<connection_types::should_use_value_message<T>::value>::type* = 0)
    {
        static_assert(!std::is_base_of<TokenData, T>::value, "not applicable to messages");
        for (const TokenDataConstPtr& td : i->value) {
            res->push_back(std::dynamic_pointer_cast<GenericValueMessage<T> const>(td)->value);
        }
    }

    template <typename T>
    void set(const std::shared_ptr<std::vector<T>>& v)
    {
        if (auto impl = std::dynamic_pointer_cast<Implementation<T>>(pimpl)) {
            impl->value = v;
        } else {
            throw std::runtime_error("cannot set the vector");
        }
    }

    void encode(YAML::Node& node) const
    {
        node["value_type"] = pimpl->nestedName();
        pimpl->encode(node);
    }

    void decode(const YAML::Node& node)
    {
        std::string type = node["value_type"].as<std::string>();
        pimpl = SupportedTypes::make(type);
        apex_assert_hard(pimpl);

        pimpl->decode(node);
    }

    virtual bool canConnectTo(const TokenData* other_side) const override;
    virtual bool acceptsConnectionFrom(const TokenData* other_side) const override;

    virtual std::string descriptiveName() const override;

    bool isContainer() const override
    {
        return true;
    }
    TokenData::Ptr nestedType() const override
    {
        return pimpl->nestedType();
    }

    virtual void addNestedValue(const TokenData::ConstPtr& msg) override
    {
        pimpl->addNestedValue(msg);
    }
    virtual TokenData::ConstPtr nestedValue(std::size_t i) const override
    {
        return pimpl->nestedValue(i);
    }
    virtual std::size_t nestedValueCount() const override
    {
        return pimpl->nestedValueCount();
    }

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override
    {
        data << pimpl->nestedName();

        pimpl->serialize(data, version);
    }
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override
    {
        std::string type;
        data >> type;
        pimpl = SupportedTypes::make(type);
        apex_assert_hard(pimpl);

        pimpl->deserialize(data, version);
    }

    static GenericVectorMessage::Ptr makeEmpty()
    {
        return std::shared_ptr<GenericVectorMessage>(new GenericVectorMessage);
    }

private:
    GenericVectorMessage(EntryInterface::Ptr pimpl, const std::string& frame_id, Message::Stamp stamp_micro_seconds);

    GenericVectorMessage();

private:
    EntryInterface::Ptr pimpl;
};

template <>
struct type<GenericVectorMessage>
{
    static std::string name()
    {
        return "Vector";
    }
};
}  // namespace connection_types

template <>
inline std::shared_ptr<connection_types::GenericVectorMessage> makeEmpty<connection_types::GenericVectorMessage>()
{
    return connection_types::GenericVectorMessage::make<connection_types::GenericVectorMessage::Anything>();
}

}  // namespace csapex

template <typename T>
struct GenericVectorRegistered
{
    GenericVectorRegistered()
    {
        csapex::connection_types::GenericVectorMessage::registerType<T>();
    }

    ~GenericVectorRegistered()
    {
        csapex::connection_types::GenericVectorMessage::deregisterType<T>();
    }
};

/// YAML
namespace YAML
{
template <>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::GenericVectorMessage>
{
    static Node encode(const csapex::connection_types::GenericVectorMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::GenericVectorMessage& rhs);
};
}  // namespace YAML

#endif  // GENERIC_VECTOR_MESSAGE_HPP
