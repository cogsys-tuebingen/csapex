/// HEADER
#include <csapex/signal/slot.h>

/// COMPONENT
#include <csapex/signal/trigger.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/command.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Slot::Slot(Settings& settings, const UUID &uuid)
    : Connectable(settings, uuid), target(NULL), buffer_(new Buffer), optional_(false)
{
    QObject::connect(this, SIGNAL(gotMessage(ConnectionType::Ptr)), this, SLOT(handleMessage(ConnectionType::Ptr)), Qt::QueuedConnection);
}

Slot::Slot(Settings &settings, Unique* parent, int sub_id)
    : Connectable(settings, parent, sub_id, "slot"), target(NULL), buffer_(new Buffer), optional_(false)
{
    QObject::connect(this, SIGNAL(gotMessage(ConnectionType::Ptr)), this, SLOT(handleMessage(ConnectionType::Ptr)), Qt::QueuedConnection);
}

Slot::~Slot()
{
    if(target != NULL) {
        target->removeConnection(this);
    }

    free();
}

void Slot::reset()
{
    free();
    setSequenceNumber(0);
}

bool Slot::tryConnect(Connectable* other_side)
{
    if(!other_side->canOutput()) {
        std::cerr << "cannot connect, other side can't output" << std::endl;
        return false;
    }

    return other_side->tryConnect(this);
}

bool Slot::acknowledgeConnection(Connectable* other_side)
{
    target = dynamic_cast<Trigger*>(other_side);
    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)), Qt::DirectConnection);
    connect(other_side, SIGNAL(enabled(bool)), this, SIGNAL(connectionEnabled(bool)));
    return true;
}

void Slot::removeConnection(Connectable* other_side)
{
    if(target != NULL) {
        apex_assert_hard(other_side == target);
        target = NULL;

        Q_EMIT connectionRemoved(this);
    }
}

Command::Ptr Slot::removeAllConnectionsCmd()
{
    Command::Ptr cmd(new command::DeleteConnection(target, this));
    return cmd;
}

void Slot::setOptional(bool optional)
{
    optional_ = optional;
}

bool Slot::isOptional() const
{
    return optional_;
}

bool Slot::hasReceived() const
{
    return isConnected() && buffer_->isFilled();
}
bool Slot::hasMessage() const
{
    return hasReceived() && !buffer_->isType<connection_types::NoMessage>();
}

void Slot::stop()
{
    buffer_->disable();
    Connectable::stop();
}

void Slot::free()
{
    buffer_->free();

    setBlocked(false);
}

void Slot::enable()
{
    Connectable::enable();
    //    if(isConnected() && !getSource()->isEnabled()) {
    //        getSource()->enable();
    //    }
}

void Slot::disable()
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

void Slot::removeAllConnectionsNotUndoable()
{
    if(target != NULL) {
        target->removeConnection(this);
        target = NULL;
        setError(false);
        Q_EMIT disconnected(this);
    }
}


bool Slot::canConnectTo(Connectable* other_side, bool move) const
{
    Trigger* trigger = dynamic_cast<Trigger*>(other_side);
    return trigger;
}

bool Slot::targetsCanBeMovedTo(Connectable* other_side) const
{
    return target->canConnectTo(other_side, true) /*&& canConnectTo(getConnected())*/;
}

bool Slot::isConnected() const
{
    return target != NULL;
}

void Slot::connectionMovePreview(Connectable *other_side)
{
    Q_EMIT(connectionInProgress(getSource(), other_side));
}


void Slot::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        ConnectionType::ConstPtr target_type = target->getType();
        if(!target_type) {
            e = true;
        } else if(!target_type->canConnectTo(getType().get())) {
            e = true;
        }
    }

    setError(e);
}

Connectable *Slot::getSource() const
{
    return target;
}

void Slot::inputMessage(ConnectionType::Ptr message)
{
    Q_EMIT gotMessage(message);
}

void Slot::handleMessage(ConnectionType::Ptr message)
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

    setBlocked(true);

    try {
        buffer_->write(message);
    } catch(const std::exception& e) {
        std::cerr << getUUID() << ": writing message failed: " << e.what() << std::endl;
        throw e;
    }

    count_++;

    Q_EMIT messageArrived(this);
}

void Slot::notifyMessageProcessed()
{
    Connectable::notifyMessageProcessed();

    if(target) {
        target->notifyMessageProcessed();
    }
}
