/// HEADER
#include <csapex/msg/input.h>

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/marker_message.h>
#include <csapex/msg/output.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

Input::Input(const UUID &uuid, ConnectableOwnerWeakPtr owner)
    : Connectable(uuid, owner), transition_(nullptr), optional_(false)
{
}

Input::~Input()
{
    free();
}

int Input::maxConnectionCount() const
{
    return 1;
}

void Input::setInputTransition(InputTransition *it)
{
    transition_ = it;
}

void Input::removeInputTransition()
{
    transition_ = nullptr;
}

void Input::reset()
{
    Connectable::reset();

    free();
    setSequenceNumber(0);
}

void Input::setOptional(bool optional)
{
    optional_ = optional;
}

bool Input::isOptional() const
{
    return optional_;
}

bool Input::hasReceived() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    return message_ != nullptr;
}

bool Input::hasMessage() const
{
    if(!hasReceived()) {
        return false;
    }

    std::unique_lock<std::mutex> lock(message_mutex_);
    return !std::dynamic_pointer_cast<connection_types::MarkerMessage const>(message_->getTokenData());
}

void Input::stop()
{
    Connectable::stop();
}

void Input::free()
{
    std::unique_lock<std::mutex> lock(message_mutex_);
//    std::cerr << "clear input " << getUUID() << std::endl;
    message_.reset();
}

void Input::enable()
{
    Connectable::enable();
}

void Input::disable()
{
    Connectable::disable();

    bool needs_notify = false;
    for(ConnectionPtr& connection : connections_) {
        if(connection->holdsToken()) {
            needs_notify = true;
        }
    }

    if(message_ != nullptr) {
        free();
        needs_notify = true;
    }

    if(needs_notify) {
        notifyMessageProcessed();
    }
}

void Input::removeAllConnectionsNotUndoable()
{
    if(!connections_.empty()) {
        getSource()->removeConnection(this);
        connections_.clear();
        disconnected(shared_from_this());
    }
}

OutputPtr Input::getSource() const
{
    if(connections_.empty()) {
        return nullptr;
    } else {
        return connections_.front()->from();
    }
}

TokenPtr Input::getToken() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    return message_;
}

void Input::setToken(TokenPtr message)
{
    apex_assert_hard(message != nullptr);

    if(!std::dynamic_pointer_cast<connection_types::MarkerMessage const>(message->getTokenData())){
        int s = message->getSequenceNumber();

        //    if(s < sequenceNumber()) {
        //        std::cerr << "connector @" << getUUID().getFullName() <<
        //                     ": dropping old message @ with #" << s <<
        //                     " < #" << sequenceNumber() << std::endl;
        //        return;
        //    }

        setSequenceNumber(s);
    }

    {
        std::unique_lock<std::mutex> lock(message_mutex_);
        message_ = message;
    }
    count_++;

    message_set(this);
}

void Input::notifyMessageAvailable(Connection* connection)
{
    message_available(connection);

    if(!transition_) {
        setToken(connection->readToken());
        connection->setTokenProcessed();
    }
}

void Input::addStatusInformation(std::stringstream &status_stream) const
{
    if(TokenPtr token = getToken()) {
        status_stream << ", Last Message Type: " << token->getTokenData()->descriptiveName();
    }
    status_stream << ", optional: " << isOptional();
}
