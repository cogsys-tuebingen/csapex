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

using namespace csapex;

Input::Input(const UUID &uuid, ConnectableOwnerWeakPtr owner)
    : Connectable(uuid, owner), transition_(nullptr), optional_(false)
{
}

Input::~Input()
{
    free();
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

bool Input::isConnectionPossible(Connectable* other_side)
{
    if(!other_side->canOutput()) {
        std::cerr << "cannot connect, other side can't output" << std::endl;
        return false;
    }

    return other_side->isConnectionPossible(this);
}

void Input::removeConnection(Connectable* other_side)
{
    if(connections_.empty()) {
        return;
    }
    connections_.clear();
    connection_removed_to(this);
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
    return isConnected() && message_ != nullptr;
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

    if(message_ != nullptr) {
        free();
        notifyMessageProcessed();
    }
}

void Input::removeAllConnectionsNotUndoable()
{
    if(!connections_.empty()) {
        getSource()->removeConnection(this);
        connections_.clear();
        setError(false);
        disconnected(this);
    }
}

bool Input::canConnectTo(Connectable* other_side, bool move) const
{
    return Connectable::canConnectTo(other_side, move) && (move || !isConnected());
}

bool Input::targetsCanBeMovedTo(Connectable* other_side) const
{
    return getSource()->canConnectTo(other_side, true) /*&& canConnectTo(getConnected())*/;
}

void Input::connectionMovePreview(Connectable *other_side)
{
    connectionInProgress(getSource().get(), other_side);
}


void Input::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        TokenData::ConstPtr target_type = getSource()->getType();
        TokenData::ConstPtr type = getType();
        if(!target_type) {
            e = true;
        } else if(!target_type->canConnectTo(type.get())) {
            e = true;
        }
    }

    setError(e);
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
