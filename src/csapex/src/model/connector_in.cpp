/// HEADER
#include <csapex/model/connector_in.h>

/// COMPONENT
#include <csapex/model/connector_out.h>
#include <csapex/command/delete_connection.h>

/// SYSTEM
#include <assert.h>
#include <iostream>

using namespace csapex;

ConnectorIn::ConnectorIn(const std::string &uuid)
    : Connectable(uuid), target(NULL), can_process(true), optional_(false), async_(false)
{
}

ConnectorIn::ConnectorIn(Unique* parent, int sub_id)
    : Connectable(parent, sub_id, TYPE_IN), target(NULL), can_process(true), optional_(false), async_(false)
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

void ConnectorIn::setAsync(bool asynch)
{
    async_ = asynch;
}

bool ConnectorIn::isAsync() const
{
    return async_;
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

void ConnectorIn::notify()
{
    {
        QMutexLocker lock(&reserve_mutex);
        can_process = true;
    }
    can_process_cond.wakeAll();
}

void ConnectorIn::wait()
{
    QMutexLocker lock(&reserve_mutex);

    while(!can_process) {
        std::cout << "warning: " << getUUID() << "can't process" << std::endl;
        can_process_cond.wait(&reserve_mutex);

        if(!can_process) {
            std::cout << "warning: called wait on a busy input connector" << std::endl;
        } else {
            std::cout << "warning: done waiting  " << getUUID() << std::endl;
        }
    }
}

void ConnectorIn::inputMessage(ConnectionType::Ptr message)
{
    wait();

    {
        QMutexLocker lock(&io_mutex);
        message_ = message;
    }

    count_++;

    Q_EMIT messageArrived(this);
}

bool ConnectorIn::hasMessage() const
{
    return message_;
}

ConnectionType::Ptr ConnectorIn::getMessage()
{
    QMutexLocker lock(&io_mutex);
    return message_;
}
