/// HEADER
#include <csapex/msg/input.h>

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/command.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Input::Input(Transition* transition, const UUID &uuid)
    : Connectable(uuid), transition_(transition), buffer_(new Buffer), optional_(false)
{
    apex_assert_hard(transition != nullptr);
    QObject::connect(this, SIGNAL(gotMessage(ConnectionType::ConstPtr)), this, SLOT(handleMessage(ConnectionType::ConstPtr)), Qt::QueuedConnection);
}

Input::Input(Transition *transition, Unique* parent, int sub_id)
    : Connectable(parent, sub_id, "in"), transition_(transition), buffer_(new Buffer), optional_(false)
{
    apex_assert_hard(transition != nullptr);
    QObject::connect(this, SIGNAL(gotMessage(ConnectionType::ConstPtr)), this, SLOT(handleMessage(ConnectionType::ConstPtr)), Qt::QueuedConnection);
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
    apex_assert_hard(connections_.front().lock()->from() == other_side);

    connections_.clear();
    Q_EMIT connectionRemoved(this);
}

Command::Ptr Input::removeAllConnectionsCmd()
{
    if(connections_.empty()) {
        return nullptr;
    }
    apex_assert_hard(connections_.size() == 1);
    Command::Ptr cmd(new command::DeleteConnection(connections_.front().lock()->from(), this));
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
    return isConnected() && buffer_->isFilled();
}

bool Input::hasMessage() const
{
    return hasReceived() && !buffer_->containsNoMessage();
}

void Input::stop()
{
    buffer_->disable();
    Connectable::stop();
}

void Input::free()
{
    buffer_->free();

    setBlocked(false);
}

void Input::enable()
{
    Connectable::enable();
    //    if(isConnected() && !getSource()->isEnabled()) {
    //        getSource()->enable();
    //    }
}

void Input::disable()
{
    Connectable::disable();

    //    if(isBlocked()) {
    free();
    notifyMessageProcessed();
    //    }
    //    if(isConnected() && getSource()->isEnabled()) {
    //        getSource()->disable();
    //    }
}

void Input::removeAllConnectionsNotUndoable()
{
    if(!connections_.empty()) {
        apex_assert_hard(connections_.size() == 1);

        connections_.front().lock()->from()->removeConnection(this);
        connections_.clear();
        setError(false);
        Q_EMIT disconnected(this);
    }
}


bool Input::canConnectTo(Connectable* other_side, bool move) const
{
    return Connectable::canConnectTo(other_side, move) && (move || !isConnected());
}

bool Input::targetsCanBeMovedTo(Connectable* other_side) const
{
    apex_assert_hard(connections_.size() == 1);
    return connections_.front().lock()->from()->canConnectTo(other_side, true) /*&& canConnectTo(getConnected())*/;
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

        ConnectionType::ConstPtr target_type = connections_.front().lock()->from()->getType();
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

void Input::inputMessage(ConnectionType::ConstPtr message)
{
    assert(!isBlocked());
    setBlocked(true);

    Q_EMIT gotMessage(message);
}

ConnectionTypeConstPtr Input::getMessage() const
{
    return buffer_->read();
}

void Input::handleMessage(ConnectionType::ConstPtr message)
{
    if(!isEnabled()) {
        return;
    }


    int s = message->sequenceNumber();
    if(s < sequenceNumber()) {
        std::cerr << "connector @" << getUUID().getFullName() <<
                     ": dropping old message @ with #" << s <<
                     " < #" << sequenceNumber() << std::endl;
        return;
    }

    setSequenceNumber(s);

    try {
        buffer_->write(message);
    } catch(const std::exception& e) {
        std::cerr << getUUID() << ": writing message failed: " << e.what() << std::endl;
        throw e;
    }

    count_++;

    Q_EMIT messageArrived(this);
}

void Input::notifyMessageProcessed()
{

    Connectable::notifyMessageProcessed();

    if(isConnected()) {
        apex_assert_hard(connections_.size() == 1);

        connections_.front().lock()->from()->notifyMessageProcessed();
    }
}
/// MOC
#include "../../include/csapex/msg/moc_input.cpp"
