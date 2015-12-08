#ifndef MSG_IO_H
#define MSG_IO_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/msg/message_traits.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/shared_ptr_tools.hpp>

namespace csapex
{

namespace msg
{

/// COMMON
bool hasMessage(Input* input);
bool hasMessage(Output* output);

bool isConnected(Input* input);
bool isConnected(Output* output);

void enable(Input* input);
void disable(Input* input);
void enable(Output* output);
void disable(Output* output);

UUID getUUID(Input* input);
UUID getUUID(Output* input);

void setLabel(Input* input, const std::string& label);
void setLabel(Output* input, const std::string& label);

void throwError(const ConnectionTypeConstPtr& msg, const std::type_info& type);



/// INPUT
ConnectionTypeConstPtr getMessage(Input* input);

template <typename R>
std::shared_ptr<R const>
getMessage(Input* input,
           typename std::enable_if<std::is_base_of<ConnectionType, R>::value >::type* /*dummy*/ = 0)
{
    auto msg = getMessage(input);
    typename std::shared_ptr<R const> result = std::dynamic_pointer_cast<R const> (msg);
    if(!result) {
        throwError(msg, typeid(R));
    }
    return result;
}

template <typename R>
std::shared_ptr<R const>
getMessage(Input* input,
           typename std::enable_if<!std::is_base_of<ConnectionType, R>::value >::type* /*dummy*/ = 0)
{
    auto msg = getMessage(input);
    auto result = std::dynamic_pointer_cast<connection_types::GenericPointerMessage<R> const> (msg);
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
    typename std::shared_ptr<Container const> result = std::dynamic_pointer_cast<Container const> (msg);
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
    return std::dynamic_pointer_cast<R>(msg->clone());
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

template <typename T>
bool isMessage(Input* input) {
    auto msg = getMessage(input);
    auto test = std::dynamic_pointer_cast<T const> (msg);
    return test != nullptr;
}
template <typename R>
bool isValue(Input* input) {
    return isMessage< connection_types::GenericValueMessage<R> >(input);
}


/// OUTPUT
void publish(Output* output, ConnectionTypeConstPtr message);

template <typename T>
void publish(Output* output,
             typename std::shared_ptr<T> message,
             std::string frame_id = "/",
             typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
    typename connection_types::GenericPointerMessage<T>::Ptr msg(new connection_types::GenericPointerMessage<T>(frame_id));
    msg->value = message;
    publish(output, std::dynamic_pointer_cast<ConnectionType>(msg));
}

template <typename T>
void publish(Output* output,
             typename boost::shared_ptr<T> message,
             std::string frame_id = "/",
             typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
    typename connection_types::GenericPointerMessage<T>::Ptr msg(new connection_types::GenericPointerMessage<T>(frame_id));
    msg->value = shared_ptr_tools::to_std_shared(message);
    publish(output, std::dynamic_pointer_cast<ConnectionType>(msg));
}

template <typename T>
void publish(Output* output,
             T message,
             std::string frame_id = "/",
             typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0) {
    typename connection_types::GenericValueMessage<T>::Ptr msg(new connection_types::GenericValueMessage<T>(frame_id));
    msg->value = message;
    publish(output, std::dynamic_pointer_cast<ConnectionType>(msg));
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

