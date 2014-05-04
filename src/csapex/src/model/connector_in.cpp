/// HEADER
#include <csapex/model/connector_in.h>

/// COMPONENT
#include <csapex/model/connector_out.h>
#include <csapex/command/delete_connection.h>

/// SYSTEM
#include <assert.h>
#include <iostream>

using namespace csapex;

ConnectorIn::ConnectorIn(Settings& settings, const UUID &uuid)
    : Connectable(settings, uuid), target(NULL), buffer_(new Buffer(1)), optional_(false)
{
}

ConnectorIn::ConnectorIn(Settings &settings, Unique* parent, int sub_id)
    : Connectable(settings, parent, sub_id, TYPE_IN), target(NULL), buffer_(new Buffer(1)), optional_(false)
{
}

ConnectorIn::~ConnectorIn()
{
    if(target != NULL) {
        target->removeConnection(this);
    }

    free();
}

bool ConnectorIn::tryConnect(Connectable* other_side)
{
    if(!other_side->canOutput()) {
        std::cerr << "cannot connect, other side can't output" << std::endl;
        return false;
    }

    return other_side->tryConnect(this);
}

bool ConnectorIn::acknowledgeConnection(Connectable* other_side)
{
    target = dynamic_cast<ConnectorOut*>(other_side);
    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));
    return true;
}

void ConnectorIn::removeConnection(Connectable* other_side)
{
    if(target != NULL) {
        assert(other_side == target);
        target = NULL;

        Q_EMIT connectionRemoved();
    }
}

Command::Ptr ConnectorIn::removeAllConnectionsCmd()
{
    Command::Ptr cmd(new command::DeleteConnection(target, this));
    return cmd;
}

void ConnectorIn::setOptional(bool optional)
{
    optional_ = optional;
}

bool ConnectorIn::isOptional() const
{
    return optional_;
}

bool ConnectorIn::hasMessage() const
{
    return buffer_->isFilled();
}

void ConnectorIn::free()
{
    buffer_->free();

    setBlocked(false);
}

void ConnectorIn::disable()
{
    //free();
    Connectable::disable();
}

void ConnectorIn::removeAllConnectionsNotUndoable()
{
    if(target != NULL) {
        target->removeConnection(this);
        target = NULL;
        setError(false);
        Q_EMIT disconnected(this);
    }
}


bool ConnectorIn::canConnectTo(Connectable* other_side, bool move) const
{
    return Connectable::canConnectTo(other_side, move) && (move || !isConnected());
}

bool ConnectorIn::targetsCanBeMovedTo(Connectable* other_side) const
{
    return target->canConnectTo(other_side, true) /*&& canConnectTo(getConnected())*/;
}

bool ConnectorIn::isConnected() const
{
    return target != NULL;
}

void ConnectorIn::connectionMovePreview(Connectable *other_side)
{
    Q_EMIT(connectionInProgress(getSource(), other_side));
}


void ConnectorIn::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        if(!target->getType()->canConnectTo(getType().get())) {
            e = true;
        }
    }

    setError(e);
}

Connectable *ConnectorIn::getSource() const
{
    return target;
}

void ConnectorIn::inputMessage(ConnectionType::Ptr message)
{
    int s = message->sequenceNumber();
    if(s < sequenceNumber()) {
        std::cerr << "connector @" << getUUID().getFullName() <<
                     ": dropping old message @ with #" << s <<
                     " < #" << sequenceNumber() << std::endl;
        return;
    }

    setBlocked(true);

    buffer_->write(message);
    setSequenceNumber(s);

    count_++;

    Q_EMIT messageArrived(this);
}
