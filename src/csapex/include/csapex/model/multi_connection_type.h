#ifndef MULTI_CONNECTION_TYPE_H
#define MULTI_CONNECTION_TYPE_H

/// COMPONENT
#include <csapex/model/token.h>
#include <csapex/msg/message_traits.h>
#include <csapex/msg/msg_fwd.h>

/// SYSTEM
#include <vector>
#include <type_traits>


namespace csapex
{

class MultiToken : public Token
{
public:
    typedef std::shared_ptr<MultiToken> Ptr;

public:
    MultiToken(const std::vector<Token::Ptr>& types);

    virtual bool canConnectTo(const Token* other_side) const override;
    virtual bool acceptsConnectionFrom(const Token *other_side) const override;

public:
    virtual Token::Ptr clone() const override;
    virtual Token::Ptr toType() const override;

private:
    std::vector<Token::Ptr> types_;
};


namespace multi_type {
namespace detail {

template <typename... Ts>
struct AddType;

template < typename T, typename... Ts >
struct AddType<T, Ts...>
{
    template <typename MsgType>
    static void insert(std::vector<Token::Ptr>& types,
                     typename std::enable_if<connection_types::should_use_pointer_message<MsgType>::value >::type* = 0)
    {
        static_assert(IS_COMPLETE(connection_types::GenericPointerMessage<MsgType>),
                      "connection_types::GenericPointerMessage is not included: "
                      "#include <csapex/msg/generic_pointer_message.hpp>");
        types.push_back(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<MsgType> >());
    }

    template <typename MsgType>
    static void insert(std::vector<Token::Ptr>& types,
                     typename std::enable_if<connection_types::should_use_value_message<MsgType>::value >::type* = 0)
    {
        static_assert(IS_COMPLETE(connection_types::GenericValueMessage<MsgType>),
                      "connection_types::GenericPointerMessage is not included: "
                      "#include <csapex/msg/generic_pointer_message.hpp>");
        types.push_back(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >());
    }

    template <typename MsgType>
    static void insert(std::vector<Token::Ptr>& types,
                     typename std::enable_if<
                     !connection_types::should_use_pointer_message<MsgType>::value &&
                     !connection_types::should_use_value_message<MsgType>::value>::type* = 0)
    {
        types.push_back(connection_types::makeEmptyMessage<MsgType>());
    }

    static void call(std::vector<Token::Ptr>& types)
    {
        insert<T>(types);
        AddType<Ts...>::call(types);
    }
};
template <>
struct AddType<>
{
    static void call(std::vector<Token::Ptr>& /*types*/)
    {
    }
};

} // namespace detail



template <typename... Types>
static Token::Ptr make()
{
    std::vector<Token::Ptr> types;
    detail::AddType<Types...>::call(types);
    return MultiToken::Ptr(new MultiToken(types));
}
}
}


#endif // MULTI_CONNECTION_TYPE_H
