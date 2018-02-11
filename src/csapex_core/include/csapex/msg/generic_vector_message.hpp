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
#include <csapex/utility/assert.h>

/// SYSTEM
#include <string>
#include <boost/static_assert.hpp>
#include <vector>
#undef NDEBUG
#include <assert.h>

// TODO REMOVE
#include <iostream>

namespace YAML
{
template<typename T, typename S>
struct as_if;
}


namespace csapex {
namespace connection_types {

template <typename Type>
struct GenericPointerMessage;
template <typename Type>
struct GenericValueMessage;

class CSAPEX_CORE_EXPORT GenericVectorMessage : public Message
{
    friend class YAML::as_if<GenericVectorMessage, void>;

public:
    struct Anything {};

private:
    struct CSAPEX_CORE_EXPORT EntryInterface : public Message
    {
        typedef std::shared_ptr< EntryInterface > Ptr;

        EntryInterface(const std::string& name, Message::Stamp stamp = 0)
            : Message(name, "/", stamp)
        {
        }

        virtual TokenData::Ptr clone() const override
        {
            return cloneEntry();
        }

        virtual EntryInterface::Ptr cloneEntry() const = 0;

        virtual void encode(YAML::Node& node) const = 0;
        virtual void decode(const YAML::Node& node) = 0;
    };

    template <typename T>
    struct Implementation : public EntryInterface
    {
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
        typedef std::shared_ptr< Self > Ptr;


    public:
        static typename Self::Ptr make() {
            return Self::Ptr (new Self);
        }

        Implementation()
            : EntryInterface(std::string("std::vector<") + type2nameWithoutNamespace(typeid(T)) + ">")
        {
            static_assert(!std::is_same<T, void*>::value, "void* not allowed");
            value.reset(new std::vector<Payload>);
        }

        virtual EntryInterface::Ptr cloneEntry() const override
        {
            Self::Ptr r(new Self);
            r->value = std::make_shared<std::vector<Payload>>(*value);
            return r;
        }

        virtual TokenData::Ptr toType() const override
        {
            Self::Ptr r(new Self);
            return r;
        }

        virtual bool canConnectTo(const TokenData* other_side) const override
        {
            if(const EntryInterface* ei = dynamic_cast<const EntryInterface*> (other_side)) {
                return nestedType()->canConnectTo(ei->nestedType().get());
            } else {
                const GenericVectorMessage* vec = dynamic_cast<const GenericVectorMessage*> (other_side);
                if(vec != 0) {
                    return vec->canConnectTo(this);
                } else {
                    auto type = nestedType();
                    return other_side->canConnectTo(type.get());
                    //return dynamic_cast<const AnyMessage*> (other_side) != nullptr;
                }
            }
        }
        virtual bool acceptsConnectionFrom(const TokenData *other_side) const override
        {
            if(const EntryInterface* ei = dynamic_cast<const EntryInterface*> (other_side)) {
                return nestedType()->canConnectTo(ei->nestedType().get());
            } else {
                return false;
            }
        }

        void encode(YAML::Node& node) const override
        {
            std::cout << "vector<"<< typeid(Payload).name() <<" < impl" << std::endl;
            node["value_type"] = type2name(typeid(T));
            node["values"] = *value;
        }

        void decode(const YAML::Node& node) override
        {
            YAML::Emitter emitter;
            emitter << node;
            std::cout << "vector<"<< typeid(Payload).name() <<" > impl: " << emitter.c_str() << std::endl;
            value.reset(new std::vector<Payload>);
            *value = node["values"].as< std::vector<Payload> >();
        }

        TokenData::Ptr nestedType() const override
        {
            return makeTypeSwitch(Tag<Payload>());
        }

        virtual void addNestedValue(const TokenData::ConstPtr &msg) override
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
        void addCastedEntry(std::vector<std::shared_ptr<MsgType>>& ,
                            const TokenData::ConstPtr& ptr,
                            typename std::enable_if<std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            // internal: vector of msg pointers
            auto casted = std::dynamic_pointer_cast<MsgType const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(casted);
        }
        template <typename MsgType>
        void addCastedEntry(std::vector<MsgType>&,
                            const TokenData::ConstPtr& ptr,
                            typename std::enable_if<std::is_base_of<TokenData, MsgType>::value>::type* = 0)
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
        void addCastedEntry(std::vector<std::shared_ptr<MsgType>>& ,
                            const TokenData::ConstPtr& ptr,
                            typename std::enable_if<!std::is_base_of<TokenData, MsgType>::value
                            && connection_types::should_use_pointer_message<MsgType>::value>::type* = 0)
        {
            // internal: vector of non-msg values
            auto casted = std::dynamic_pointer_cast<connection_types::GenericPointerMessage<MsgType> const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(casted->value);
        }
        template <typename MsgType>
        void addCastedEntry(std::vector<MsgType>&,
                            const TokenData::ConstPtr& ptr,
                            typename std::enable_if<!std::is_base_of<TokenData, MsgType>::value
                            && connection_types::should_use_pointer_message<MsgType>::value>::type* = 0)
        {
            // internal: vector of non-msg values
            auto casted = std::dynamic_pointer_cast<connection_types::GenericPointerMessage<MsgType> const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(*casted->value);
        }
        template <typename ValueType>
        void addCastedEntry(std::vector<ValueType>&,
                            const TokenData::ConstPtr& ptr,
                            typename std::enable_if<!std::is_base_of<TokenData, ValueType>::value
                            && connection_types::should_use_value_message<ValueType>::value>::type* = 0)
        {
            // internal: vector of non-msg values
            auto casted = std::dynamic_pointer_cast<connection_types::GenericValueMessage<ValueType> const>(ptr);
            apex_assert_msg(casted, "message cannot be casted");
            value->push_back(casted->value);
        }




        template <typename MsgType>
        static TokenData::ConstPtr makeToken(const std::shared_ptr<MsgType>& ptr,
                                             typename std::enable_if<std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            return ptr;
        }

        template <typename MsgType>
        static TokenData::ConstPtr makeToken(const MsgType& val,
                                             typename std::enable_if<std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            return std::make_shared<MsgType>(val);
        }
        template <typename MsgType>
        static TokenData::ConstPtr makeToken(const MsgType& val,
                                             typename std::enable_if<!std::is_base_of<TokenData, MsgType>::value>::type* = 0)
        {
            return convertToken(val);
        }
        // ros message by value
        template <typename MsgType>
        static TokenData::ConstPtr convertToken(const MsgType& val,
                                                typename std::enable_if<connection_types::should_use_pointer_message<MsgType>::value >::type* = 0)
        {
            auto res = connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<MsgType> >();
            res->value = std::make_shared<MsgType>(val);
            return res;
        }
        // ros message by pointer
        template <typename MsgType>
        static TokenData::ConstPtr convertToken(const std::shared_ptr<MsgType>& val,
                                                typename std::enable_if<connection_types::should_use_pointer_message<MsgType>::value >::type* = 0)
        {
            auto res = connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<MsgType> >();
            res->value = val;
            return res;
        }
        // value message
        template <typename MsgType>
        static TokenData::ConstPtr convertToken(const MsgType& val,
                                                typename std::enable_if<connection_types::should_use_value_message<MsgType>::value >::type* = 0)
        {
            auto res = connection_types::makeEmptyMessage<connection_types::GenericValueMessage<MsgType> >();
            res->value = val;
            return res;
        }


        template <class Type>
        struct Tag {};


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
        static TokenData::Ptr makeTypeImpl(typename std::enable_if<connection_types::should_use_pointer_message<MsgType>::value >::type* = 0)
        {
            return connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<MsgType> >();
        }

        template <typename MsgType>
        static TokenData::Ptr makeTypeImpl(typename std::enable_if<connection_types::should_use_value_message<MsgType>::value >::type* = 0)
        {
            return connection_types::makeEmptyMessage<connection_types::GenericValueMessage<MsgType> >();
        }

        template <typename MsgType>
        static TokenData::Ptr makeTypeImpl(typename std::enable_if<
                                           !connection_types::should_use_pointer_message<MsgType>::value &&
                                           !connection_types::should_use_value_message<MsgType>::value>::type* = 0)
        {
            static_assert(std::is_base_of<TokenData, MsgType>::value, "message has to be derived from TokenData");
            return connection_types::makeEmptyMessage<MsgType>();
        }

    public:
        std::shared_ptr< std::vector<Payload> > value;
    };

    template <typename T>
    struct MessageImplementation : public Implementation<T>
    {
        typedef Implementation<T> Parent;
        typedef MessageImplementation<T> Self;

        using Parent::value;

    public:
        typedef std::shared_ptr< Self > Ptr;

        static typename Self::Ptr make() {
            return Self::Ptr (new Self);
        }

        virtual EntryInterface::Ptr cloneEntry() const override
        {
            auto r = std::make_shared<MessageImplementation<T>>();
            *r->value = *value;
            return r;
        }


        void encode(YAML::Node& node) const override
        {
            std::cout << "encode message" << std::endl;
            node["values"] = YAML::Node(YAML::NodeType::Sequence);
            for(const auto& entry : *value) {
                node["values"].push_back(MessageSerializer::serializeMessage(entry));
            }
        }
        void decode(const YAML::Node& node) override
        {
            std::cout << "decode message" << std::endl;
            YAML::Emitter emitter;
            emitter << node;
            std::cout << emitter.c_str() << std::endl;
            for(const YAML::Node& entry : node["values"]) {
                YAML::Emitter emitter;
                emitter << entry;
                std::cout << emitter.c_str() << std::endl;
                auto msg = std::dynamic_pointer_cast<T>(MessageSerializer::deserializeMessage(entry));
                value->push_back(*msg);
            }
        }
    };

    struct CSAPEX_CORE_EXPORT AnythingImplementation : public EntryInterface
    {
        AnythingImplementation();

        virtual EntryInterface::Ptr cloneEntry() const override;
        virtual TokenData::Ptr toType() const override;
        virtual bool canConnectTo(const TokenData* other_side) const override;
        virtual bool acceptsConnectionFrom(const TokenData *other_side) const override;
        virtual void encode(YAML::Node& node) const override;
        virtual void decode(const YAML::Node& node) override;
    };

    struct CSAPEX_CORE_EXPORT InstancedImplementation : public EntryInterface
    {
        friend class GenericVectorMessage;

        InstancedImplementation(TokenData::ConstPtr type);

        virtual EntryInterface::Ptr cloneEntry() const override;
        virtual TokenData::Ptr toType() const override;
        virtual bool canConnectTo(const TokenData* other_side) const override;
        virtual bool acceptsConnectionFrom(const TokenData *other_side) const override;
        virtual void encode(YAML::Node& node) const override;
        virtual void decode(const YAML::Node& node) override;

        TokenData::Ptr nestedType() const override;

        virtual void addNestedValue(const TokenData::ConstPtr &msg) override;
        virtual TokenData::ConstPtr nestedValue(std::size_t i) const override;
        virtual std::size_t nestedValueCount() const override;

    private:
        TokenData::ConstPtr type_;
        std::vector<TokenDataPtr> value;
    };

public:
    typedef std::shared_ptr<GenericVectorMessage> Ptr;
    typedef std::shared_ptr<const GenericVectorMessage> ConstPtr;

    template <typename T>
    struct TypeMap {
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



    struct SupportedTypes : public Singleton<SupportedTypes> {
        static EntryInterface::Ptr make(const std::string& type) {
            auto pos = instance().map_.find(type);
            if(pos == instance().map_.end()) {
                for(auto pair : instance().map_) {
                    std::cout << "supported: " << pair.first << std::endl;
                }
                throw std::runtime_error(std::string("cannot make vector of type ") + type);
            }
            std::cout << "!!!! make vector of type " << type << ": " << pos->second->typeName() << std::endl;;
            return pos->second->cloneEntry();
        }

        template <typename T>
        static void registerType(const std::string& type_name = "")
        {
            std::string type = type_name.empty() ? type2name(typeid(T)) : type_name;
            std::map<std::string, EntryInterface::Ptr>& map = instance().map_;
            if(map.find(type) == map.end()) {
                Adder<T>::addToMap(instance().map_, type);
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
    static GenericVectorMessage::Ptr make(typename std::enable_if<std::is_base_of<TokenData, T>::value >::type* dummy = 0)
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
    static GenericVectorMessage::Ptr make(typename std::enable_if<std::is_same<Anything, T>::value >::type* dummy = 0)
    {
        return GenericVectorMessage::Ptr(new GenericVectorMessage(std::make_shared<AnythingImplementation>(), "/", 0));
    }


    static GenericVectorMessage::Ptr make(TokenData::ConstPtr type)
    {
        return GenericVectorMessage::Ptr(new GenericVectorMessage(std::make_shared<InstancedImplementation>(type), "/", 0));
    }


    template <typename T>
    std::shared_ptr<std::vector<T> const>  makeShared(typename std::enable_if<std::is_base_of<TokenData, T>::value>::type* = 0) const
    {
        if(auto i = std::dynamic_pointer_cast< Implementation<T> > (impl)) {
            return i->value;
        } else if(auto i = std::dynamic_pointer_cast< InstancedImplementation > (impl)) {
            auto res = std::make_shared<std::vector<T>>();
            for(TokenDataConstPtr td : i->value) {
                if(auto v = std::dynamic_pointer_cast<T const>(td)) {
                    res->push_back(*v);
                }
            }
            return res;
        } else {
            std::cout << "type is " << type2name<T>() << std::endl;
            throw std::runtime_error("cannot make the msg vector shared");
        }
    }
    template <typename T>
    std::shared_ptr<std::vector<T> const>  makeShared(typename std::enable_if<!std::is_base_of<TokenData, T>::value>::type* = 0) const
    {
        if(auto i = std::dynamic_pointer_cast< Implementation<T> > (impl)) {
            return i->value;
        } else if(auto i = std::dynamic_pointer_cast< InstancedImplementation > (impl)) {
            auto res = std::make_shared<std::vector<T>>();
            makeSharedValue(i.get(), res);
            return res;
        } else if(auto i = std::dynamic_pointer_cast< MessageImplementation<GenericValueMessage<T>> >  (impl)) {
            auto res = std::make_shared<std::vector<T>>();
            for(auto entry : *i->value) {
                res->push_back(entry.value);
            }
            return res;
        } else {
            std::cout << "instance is " << type2name(typeid(*impl)) << std::endl;
            std::cout << "type is " << type2name<T>() << std::endl;
            throw std::runtime_error("cannot make the direct vector shared");
        }
    }

    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<T>>& res,
                                typename std::enable_if<std::is_base_of<TokenData, T>::value >::type* = 0)
    {
        for(TokenDataConstPtr td : i->value) {
            res->push_back(*std::dynamic_pointer_cast<T const>(td));
        }
    }
    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<std::shared_ptr<T>>>& res,
                                typename std::enable_if<std::is_base_of<TokenData, T>::value >::type* = 0)
    {
        for(TokenDataConstPtr td : i->value) {
            res->push_back(std::dynamic_pointer_cast<T>(td));
        }
    }
    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<std::shared_ptr<T const>>>& res,
                                typename std::enable_if<std::is_base_of<TokenData, T>::value >::type* = 0)
    {
        for(TokenDataConstPtr td : i->value) {
            res->push_back(std::dynamic_pointer_cast<T const>(td));
        }
    }

    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<T>>& res,
                                typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0)
    {
        static_assert(!std::is_base_of<TokenData, T>::value, "not applicable to messages");
        for(TokenDataConstPtr td : i->value) {
            res->push_back(*std::dynamic_pointer_cast<GenericPointerMessage<T> const>(td)->value);
        }
    }
    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<std::shared_ptr<T>>>& res,
                                typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0)
    {
        static_assert(!std::is_base_of<TokenData, T>::value, "not applicable to messages");
        for(TokenDataConstPtr td : i->value) {
            res->push_back(std::dynamic_pointer_cast<GenericPointerMessage<T> const>(td)->value);
        }
    }
    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<std::shared_ptr<T const>>>& res,
                                typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0)
    {
        static_assert(!std::is_base_of<TokenData, T>::value, "not applicable to messages");
        for(TokenDataConstPtr td : i->value) {
            res->push_back(std::dynamic_pointer_cast<GenericPointerMessage<T const> const>(td)->value);
        }
    }

    template <typename T>
    static void makeSharedValue(InstancedImplementation* i, std::shared_ptr<std::vector<T>>& res,
                                typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0)
    {
        static_assert(!std::is_base_of<TokenData, T>::value, "not applicable to messages");
        for(TokenDataConstPtr td : i->value) {
            res->push_back(std::dynamic_pointer_cast<GenericValueMessage<T> const>(td)->value);
        }
    }


    template <typename T>
    void set(const std::shared_ptr< std::vector<T> > & v) {
        if(auto i = std::dynamic_pointer_cast< Implementation<T> > (impl)) {
            i->value = v;
        } else {
            throw std::runtime_error("cannot set the vector");
        }
    }

    void encode(YAML::Node& node) const
    {
        impl->encode(node);
    }

    void decode(const YAML::Node& node)
    {
        std::string type = node["value_type"].as<std::string>();
        impl = SupportedTypes::make(type);
        assert(impl);

        impl->decode(node);
    }


    virtual TokenData::Ptr clone() const override;
    virtual TokenData::Ptr toType() const override;

    virtual bool canConnectTo(const TokenData* other_side) const override;
    virtual bool acceptsConnectionFrom(const TokenData *other_side) const override;

    virtual std::string descriptiveName() const override;


    bool isContainer() const override
    {
        return true;
    }
    TokenData::Ptr nestedType() const override
    {
        return impl->nestedType();
    }

    virtual void addNestedValue(const TokenData::ConstPtr &msg) override
    {
        impl->addNestedValue(msg);
    }
    virtual TokenData::ConstPtr nestedValue(std::size_t i) const override
    {
        return impl->nestedValue(i);
    }
    virtual std::size_t nestedValueCount() const override
    {
        return impl->nestedValueCount();
    }

private:
    GenericVectorMessage(EntryInterface::Ptr impl, const std::string &frame_id, Message::Stamp stamp_micro_seconds);

    GenericVectorMessage();

private:
    EntryInterface::Ptr impl;

};

template <>
struct type<GenericVectorMessage> {
    static std::string name() {
        return "Vector";
    }
};

template <>
inline std::shared_ptr<GenericVectorMessage> makeEmpty<GenericVectorMessage>()
{
    return GenericVectorMessage::make<GenericVectorMessage::Anything>();
}

}
}

template <typename T>
struct GenericVectorRegistered
{
    GenericVectorRegistered() {
        csapex::connection_types::GenericVectorMessage::registerType<T>();
    }
};


/// YAML
namespace YAML {
template<>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::GenericVectorMessage> {
    static Node encode(const csapex::connection_types::GenericVectorMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::GenericVectorMessage& rhs);
};
}

#endif // GENERIC_VECTOR_MESSAGE_HPP
