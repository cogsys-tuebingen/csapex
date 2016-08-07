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

namespace csapex {
namespace connection_types {

struct GenericVectorMessage : public Message
{
public:
    struct Anything {};

private:
    struct EntryInterface : public Message
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
        static Self::Ptr make() {
            return Self::Ptr (new Self);
        }

        Implementation()
            : EntryInterface(std::string("std::vector<") + type2nameWithoutNamespace(typeid(T)) + ">")
        {
            static_assert(!std::is_same<T, void*>::value, "void* not allowed");
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
                    return dynamic_cast<const AnyMessage*> (other_side) != nullptr;
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
            node["value_type"] = type2name(typeid(T));
            node["values"] = *value;
        }

        void decode(const YAML::Node& node) override
        {
            value.reset(new std::vector<Payload>);
            *value = node["values"].as< std::vector<Payload> >();
        }

        TokenData::Ptr nestedType() const override
        {
            return makeTypeSwitch(Payload());
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





        template <typename MsgType>
        static TokenData::Ptr makeTypeSwitch(const std::shared_ptr<MsgType>& ptr)
        {
            return makeTypeImpl<MsgType>();
        }
        template <typename MsgType>
        static TokenData::Ptr makeTypeSwitch(const MsgType& type)
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

        static Self::Ptr make() {
            return Self::Ptr (new Self);
        }
    };

    struct AnythingImplementation : public EntryInterface
    {
        AnythingImplementation();

        virtual EntryInterface::Ptr cloneEntry() const;
        virtual TokenData::Ptr toType() const override;
        virtual bool canConnectTo(const TokenData* other_side) const override;
        virtual bool acceptsConnectionFrom(const TokenData *other_side) const override;
        virtual void encode(YAML::Node& node) const;
        virtual void decode(const YAML::Node& node);
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

    struct SupportedTypes {
        static SupportedTypes& instance() {
            static SupportedTypes i;
            return i;
        }
        static EntryInterface::Ptr make(const std::string& type) {
            return instance().map_.at(type)->cloneEntry();
        }

        template <typename T>
        static void registerType()
        {
            std::string type = type2name(typeid(T));
            std::map<std::string, EntryInterface::Ptr>& map = instance().map_;
            if(map.find(type) == map.end()) {
                map[type].reset(new Implementation<T>());
            }
        }

    private:
        std::map<std::string, EntryInterface::Ptr> map_;
    };

    template <typename T>
    static void registerType()
    {
        SupportedTypes::registerType<T>();
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

    template <typename T>
    std::shared_ptr<std::vector<T> const>  makeShared() const
    {
        return std::dynamic_pointer_cast< Implementation<T> > (impl)->value;
    }


    template <typename T>
    void set(const std::shared_ptr< std::vector<T> > & v) {
        std::dynamic_pointer_cast< Implementation<T> > (impl)->value = v;
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


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::GenericVectorMessage> {
    static Node encode(const csapex::connection_types::GenericVectorMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::GenericVectorMessage& rhs);
};
}

#endif // GENERIC_VECTOR_MESSAGE_HPP
