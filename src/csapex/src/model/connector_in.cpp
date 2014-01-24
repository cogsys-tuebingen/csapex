/// HEADER
#include <csapex/model/connector_in.h>

/// COMPONENT
#include <csapex/model/connector_out.h>
#include <csapex/command/delete_connection.h>

/// SYSTEM
#include <assert.h>
#include <iostream>

using namespace csapex;

ConnectorIn::ConnectorIn(const UUID &uuid)
    : Connectable(uuid), target(NULL), optional_(false), legacy_(true)
{
}

ConnectorIn::ConnectorIn(Unique* parent, int sub_id)
    : Connectable(parent, sub_id, TYPE_IN), target(NULL), optional_(false), legacy_(true)
{
}

ConnectorIn::~ConnectorIn()
{
    if(target != NULL) {
        target->removeConnection(this);
    }
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

void ConnectorIn::setLegacy(bool legacy)
{
    legacy_ = legacy;
}

bool ConnectorIn::isLegacy() const
{
    return legacy_;
}


bool ConnectorIn::hasMessage() const
{
    return message_;
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

void ConnectorIn::waitForProcessing(const UUID& who_is_waiting)
{
    if(isAsync() || !isEnabled()) {
        return;
    }

    Connectable::waitForProcessing(who_is_waiting);
}

void ConnectorIn::setProcessing(bool processing)
{
//    // call parents
//    if(isConnected()) {
//        if(!isAsync() || isProcessing() != processing) {
//            Connectable* parent = getSource();
//            //parent->setProcessing(processing);
//        }
//    }
    Connectable::setProcessing(processing);
}

void ConnectorIn::waitForMessage()
{
    QMutexLocker lock(&io_mutex);

//    while(!has_msg) {
//        has_msg_cond.wait(&io_mutex);
//    }
}

void ConnectorIn::updateIsProcessing()
{
    if(!isConnected() && isProcessing()) {
        setProcessing(false);
    }
}

void ConnectorIn::inputMessage(ConnectionType::Ptr message)
{
    {
        QMutexLocker lock(&io_mutex);
        message_ = message;
    }

    if(!isAsync() || (isAsync() && !isProcessing())) {
        setProcessing(true);
    }

    count_++;

    Q_EMIT messageArrived(this);
}

ConnectionType::Ptr ConnectorIn::getMessage()
{
    QMutexLocker lock(&io_mutex);
    return message_;
}
