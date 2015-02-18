/// HEADER
#include <csapex/msg/input.h>

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/command.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/transition.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Input::Input(Transition* transition, const UUID &uuid)
    : Connectable(uuid), transition_(transition), optional_(false)
{
    apex_assert_hard(transition != nullptr);
}

Input::Input(Transition *transition, Unique* parent, int sub_id)
    : Connectable(parent, sub_id, "in"), transition_(transition), optional_(false)
{
    apex_assert_hard(transition != nullptr);
}

Input::~Input()
{
    free();
}

Transition* Input::getTransition() const
{
    return transition_;
}

void Input::reset()
{
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
    apex_assert_hard(connections_.size() == 1);
    apex_assert_hard(getSource() == other_side);

    connections_.clear();
    Q_EMIT connectionRemoved(this);
}

Command::Ptr Input::removeAllConnectionsCmd()
{
    if(connections_.empty()) {
        return nullptr;
    }
    apex_assert_hard(connections_.size() == 1);
    Command::Ptr cmd(new command::DeleteConnection(getSource(), this));
    return cmd;
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
    return !std::dynamic_pointer_cast<connection_types::NoMessage const>(message_);
}

void Input::stop()
{
    Connectable::stop();
}

void Input::free()
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    message_.reset();
}

void Input::enable()
{
    Connectable::enable();
}

void Input::disable()
{
    Connectable::disable();

    free();
    notifyMessageProcessed();
}

void Input::removeAllConnectionsNotUndoable()
{
    if(!connections_.empty()) {
        apex_assert_hard(connections_.size() == 1);

        getSource()->removeConnection(this);
        connections_.clear();
        setError(false);
        Q_EMIT disconnected(this);
    }
}

void Input::addConnection(ConnectionWeakPtr connection)
{
    transition_->addConnection(connection);
    Connectable::addConnection(connection);
}

void Input::removeConnection(ConnectionWeakPtr connection)
{
    transition_->removeConnection(connection);
    Connectable::removeConnection(connection);
}

bool Input::canConnectTo(Connectable* other_side, bool move) const
{
    return Connectable::canConnectTo(other_side, move) && (move || !isConnected());
}

bool Input::targetsCanBeMovedTo(Connectable* other_side) const
{
    apex_assert_hard(connections_.size() == 1);
    return getSource()->canConnectTo(other_side, true) /*&& canConnectTo(getConnected())*/;
}

void Input::connectionMovePreview(Connectable *other_side)
{
    Q_EMIT(connectionInProgress(getSource(), other_side));
}


void Input::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        apex_assert_hard(connections_.size() == 1);

        ConnectionType::ConstPtr target_type = getSource()->getType();
        ConnectionType::ConstPtr type = getType();
        if(!target_type) {
            e = true;
        } else if(!target_type->canConnectTo(type.get())) {
            e = true;
        }
    }

    setError(e);
}

Connectable *Input::getSource() const
{
    assert(connections_.size() <= 1);
    if(connections_.empty()) {
        return nullptr;
    } else {
        return connections_.front().lock()->from();
    }
}

ConnectionTypeConstPtr Input::getMessage() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    apex_assert_hard(message_ != nullptr);
    return message_;
}

void Input::inputMessage(ConnectionType::ConstPtr message)
{
    apex_assert_hard(message != nullptr);

    if(!isEnabled()) {
        notifyMessageProcessed();

        return;
    }


    int s = message->sequenceNumber();
//    if(s < sequenceNumber()) {
//        std::cerr << "connector @" << getUUID().getFullName() <<
//                     ": dropping old message @ with #" << s <<
//                     " < #" << sequenceNumber() << std::endl;
//        return;
//    }

    setSequenceNumber(s);

    {
        std::unique_lock<std::mutex> lock(message_mutex_);
        message_ = message;
    }
    count_++;

    Q_EMIT messageArrived(this);
}

void Input::notifyMessageProcessed()
{
    Connectable::notifyMessageProcessed();

    if(isConnected()) {
        apex_assert_hard(connections_.size() == 1);
        transition_->notifyMessageProcessed();
    }
}
/// MOC
#include "../../include/csapex/msg/moc_input.cpp"
