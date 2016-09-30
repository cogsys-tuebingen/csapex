#ifndef MSG_IO_H
#define MSG_IO_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/uuid.h>

namespace boost
{
template <typename T> class shared_ptr;
}

namespace shared_ptr_tools
{
template <typename T> std::shared_ptr<T> to_std_shared(const boost::shared_ptr<T> &p);
}

namespace csapex
{
namespace msg
{

/// COMMON
CSAPEX_EXPORT bool hasMessage(Input* input);
CSAPEX_EXPORT bool hasMessage(Output* output);

CSAPEX_EXPORT bool isConnected(Input* input);
CSAPEX_EXPORT bool isConnected(Output* output);

CSAPEX_EXPORT bool isEnabled(Input* input);
CSAPEX_EXPORT bool isEnabled(Output* output);

CSAPEX_EXPORT void enable(Input* input);
CSAPEX_EXPORT void disable(Input* input);
CSAPEX_EXPORT void enable(Output* output);
CSAPEX_EXPORT void disable(Output* output);

CSAPEX_EXPORT UUID getUUID(Input* input);
CSAPEX_EXPORT UUID getUUID(Output* input);

CSAPEX_EXPORT void setLabel(Input* input, const std::string& label);
CSAPEX_EXPORT void setLabel(Output* input, const std::string& label);

CSAPEX_EXPORT void throwError(const TokenDataConstPtr& msg, const std::type_info& type);


/// CASTING

template <typename R, typename S, typename Selector = void>
struct DefaultMessageCaster
{
    static std::shared_ptr<R const> constcast(const std::shared_ptr<S const>& msg)
    {
        return std::dynamic_pointer_cast<R const>(msg);
    }
    static std::shared_ptr<R> cast(const std::shared_ptr<S>& msg)
    {
        return std::dynamic_pointer_cast<R>(msg);
    }
};

template <typename R, typename S, typename Selector = void>
struct MessageCaster
{
    static std::shared_ptr<R const> constcast(const std::shared_ptr<S const>& msg)
    {
        return DefaultMessageCaster<R,S,Selector>::constcast(msg);
    }
    static std::shared_ptr<R> cast(const std::shared_ptr<S>& msg)
    {
        return DefaultMessageCaster<R,S,Selector>::cast(msg);
    }
};

template <typename R, typename S, typename Selector = void>
std::shared_ptr<R const> message_cast(const std::shared_ptr<S const>& msg)
{
    return MessageCaster<typename std::remove_const<R>::type,typename std::remove_const<S>::type,Selector>::constcast(msg);
}
template <typename R, typename S, typename Selector = void>
std::shared_ptr<R> message_cast(const std::shared_ptr<S>& msg)
{
    return MessageCaster<typename std::remove_const<R>::type,typename std::remove_const<S>::type,Selector>::cast(msg);
}

/// INPUT
CSAPEX_EXPORT TokenDataConstPtr getMessage(Input* input);

template <typename R>
std::shared_ptr<R const>
getMessage(Input* input,
           typename std::enable_if<std::is_base_of<TokenData, R>::value >::type* /*dummy*/ = 0)
{
    auto msg = getMessage(input);
    typename std::shared_ptr<R const> result = message_cast<R const> (msg);
    if(!result) {
        throwError(msg, typeid(R));
    }
    return result;
}

template <typename R>
std::shared_ptr<R const>
getMessage(Input* input,
           typename std::enable_if<!std::is_base_of<TokenData, R>::value >::type* /*dummy*/ = 0)
{
    auto msg = getMessage(input);
    auto result = message_cast<connection_types::GenericPointerMessage<R> const> (msg);
    if(!result) {
        throwError(msg, typeid(R));
    }
    return result->value;
}

template <typename Container, typename R>
std::shared_ptr<typename Container::template TypeMap<R>::type const>
getMessage(Input* input)
{
    auto msg = getMessage(input);
    typename std::shared_ptr<Container const> result = message_cast<Container const> (msg);
    if(!result) {
        throwError(msg, typeid(Container));
    }
    return result -> template makeShared<R>();
}


template <typename R>
std::shared_ptr<R>
getClonedMessage(Input* input)
{
    const auto& msg = getMessage<R>(input);
    if(msg == nullptr) {
        return nullptr;
    }
    return message_cast<R>(msg->clone());
}


template <typename R>
R getValue(Input* input)
{
    const auto& msg = getMessage< connection_types::GenericValueMessage<R> >(input);
    if(!msg) {
        throw std::logic_error("cannot convert message to requested value");
    }
    return msg->value;
}

template <typename R>
bool isMessage(Input* input,
               typename std::enable_if<std::is_base_of<TokenData, R>::value >::type* /*dummy*/ = 0)
{
    auto msg = getMessage(input);
    auto test = message_cast<R const> (msg);
    return test != nullptr;
}

template <typename R>
bool isMessage(Input* input,
               typename std::enable_if<!std::is_base_of<TokenData, R>::value >::type* /*dummy*/ = 0)
{
    auto msg = getMessage(input);
    auto test = message_cast<connection_types::GenericPointerMessage<R> const> (msg);
    return test != nullptr;
}

template <typename R>
bool isValue(Input* input) {
    return isMessage< connection_types::GenericValueMessage<R> >(input);
}


/// OUTPUT
CSAPEX_EXPORT void publish(Output* output, TokenDataConstPtr message);

template <typename T,
          typename = typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type>
void publish(Output* output,
             typename std::shared_ptr<T> message,
             std::string frame_id = "/")
{
    typename connection_types::GenericPointerMessage<T>::Ptr msg(new connection_types::GenericPointerMessage<T>(frame_id));
    msg->value = message;
    publish(output, message_cast<TokenData>(msg));
}

template <typename T,
          typename = typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type>
void publish(Output* output,
             typename boost::shared_ptr<T> message,
             std::string frame_id = "/")
{
    typename connection_types::GenericPointerMessage<T>::Ptr msg(new connection_types::GenericPointerMessage<T>(frame_id));
    msg->value = shared_ptr_tools::to_std_shared(message);
    publish(output, message_cast<TokenData>(msg));
}

template <typename T,
          typename = typename std::enable_if<connection_types::should_use_value_message<T>::value >::type>
void publish(Output* output,
             T message,
             std::string frame_id = "/")
{
    typename connection_types::GenericValueMessage<T>::Ptr msg(new connection_types::GenericValueMessage<T>(frame_id));
    msg->value = message;
    publish(output, message_cast<TokenData>(msg));
}

template <class Container, typename T>
void publish(Output* output,
             const typename Container::template TypeMap<T>::Ptr& message) {
    typename std::shared_ptr<Container> msg(Container::template make<T>());
    msg->template set<T>(message);
    publish(output, msg);
}
}
}

#endif // MSG_IO_H

