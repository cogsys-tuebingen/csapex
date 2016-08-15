#ifndef MULTI_CONNECTION_TYPE_H
#define MULTI_CONNECTION_TYPE_H

/// COMPONENT
#include <csapex/model/token_data.h>
#include <csapex/msg/token_traits.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <vector>
#include <type_traits>


namespace csapex
{

class CSAPEX_EXPORT MultiTokenData : public TokenData
{
public:
    typedef std::shared_ptr<MultiTokenData> Ptr;

public:
    MultiTokenData(const std::vector<TokenData::Ptr>& types);

    virtual bool canConnectTo(const TokenData* other_side) const override;
    virtual bool acceptsConnectionFrom(const TokenData *other_side) const override;

public:
    virtual TokenData::Ptr clone() const override;
    virtual TokenData::Ptr toType() const override;

private:
    std::vector<TokenData::Ptr> types_;
};


namespace multi_type {
namespace detail {

template <typename... Ts>
struct AddType;

template < typename T, typename... Ts >
struct AddType<T, Ts...>
{
    template <typename MsgType>
    static void insert(std::vector<TokenData::Ptr>& types,
                     typename std::enable_if<connection_types::should_use_pointer_message<MsgType>::value >::type* = 0)
    {
        static_assert(IS_COMPLETE(connection_types::GenericPointerMessage<MsgType>),
                      "connection_types::GenericPointerMessage is not included: "
                      "#include <csapex/msg/generic_pointer_message.hpp>");
        types.push_back(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<MsgType> >());
    }

    template <typename MsgType>
    static void insert(std::vector<TokenData::Ptr>& types,
                     typename std::enable_if<connection_types::should_use_value_message<MsgType>::value >::type* = 0)
    {
        static_assert(IS_COMPLETE(connection_types::GenericValueMessage<MsgType>),
                      "connection_types::GenericPointerMessage is not included: "
                      "#include <csapex/msg/generic_pointer_message.hpp>");
        types.push_back(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >());
    }

    template <typename MsgType>
    static void insert(std::vector<TokenData::Ptr>& types,
                     typename std::enable_if<
                     !connection_types::should_use_pointer_message<MsgType>::value &&
                     !connection_types::should_use_value_message<MsgType>::value>::type* = 0)
    {
        types.push_back(connection_types::makeEmptyMessage<MsgType>());
    }

    static void call(std::vector<TokenData::Ptr>& types)
    {
        insert<T>(types);
        AddType<Ts...>::call(types);
    }
};
template <>
struct AddType<>
{
    static void call(std::vector<TokenData::Ptr>& /*types*/)
    {
    }
};

} // namespace detail



template <typename... Types>
static TokenData::Ptr make()
{
    std::vector<TokenData::Ptr> types;
    detail::AddType<Types...>::call(types);
    return MultiTokenData::Ptr(new MultiTokenData(types));
}
}
}


#endif // MULTI_CONNECTION_TYPE_H
