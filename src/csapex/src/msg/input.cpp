/// HEADER
#include <csapex/msg/input.h>

/// COMPONENT
#include <csapex/msg/output.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/command.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Input::Input(Settings& settings, const UUID &uuid)
    : Connectable(settings, uuid), target(NULL), buffer_(new Buffer), optional_(false)
{
    QObject::connect(this, SIGNAL(gotMessage(ConnectionType::Ptr)), this, SLOT(handleMessage(ConnectionType::Ptr)), Qt::QueuedConnection);
}

Input::Input(Settings &settings, Unique* parent, int sub_id)
    : Connectable(settings, parent, sub_id, TYPE_IN), target(NULL), buffer_(new Buffer), optional_(false)
{
    QObject::connect(this, SIGNAL(gotMessage(ConnectionType::Ptr)), this, SLOT(handleMessage(ConnectionType::Ptr)), Qt::QueuedConnection);
}

Input::~Input()
{
    if(target != NULL) {
        target->removeConnection(this);
    }

    free();
}

void Input::reset()
{
    free();
    setSequenceNumber(0);
}

bool Input::tryConnect(Connectable* other_side)
{
    if(!other_side->canOutput()) {
        std::cerr << "cannot connect, other side can't output" << std::endl;
        return false;
    }

    return other_side->tryConnect(this);
}

bool Input::acknowledgeConnection(Connectable* other_side)
{
    target = dynamic_cast<Output*>(other_side);
    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));
    connect(other_side, SIGNAL(enabled(bool)), this, SIGNAL(connectionEnabled(bool)));
    return true;
}

void Input::removeConnection(Connectable* other_side)
{
    if(target != NULL) {
        apex_assert_hard(other_side == target);
        target = NULL;

        Q_EMIT connectionRemoved(this);
    }
}

Command::Ptr Input::removeAllConnectionsCmd()
{
    Command::Ptr cmd(new command::DeleteConnection(target, this));
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

bool Input::hasMessage() const
{
    return isConnected() && buffer_->isFilled() && !buffer_->isType<connection_types::NoMessage>();
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
    if(target != NULL) {
        target->removeConnection(this);
        target = NULL;
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
    return target->canConnectTo(other_side, true) /*&& canConnectTo(getConnected())*/;
}

bool Input::isConnected() const
{
    return target != NULL;
}

void Input::connectionMovePreview(Connectable *other_side)
{
    Q_EMIT(connectionInProgress(getSource(), other_side));
}


void Input::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        if(!target->getType()->canConnectTo(getType().get())) {
            e = true;
        }
    }

    setError(e);
}

Connectable *Input::getSource() const
{
    return target;
}

void Input::inputMessage(ConnectionType::Ptr message)
{
    Q_EMIT gotMessage(message);
}

void Input::handleMessage(ConnectionType::Ptr message)
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

void Input::notifyMessageProcessed()
{
    Connectable::notifyMessageProcessed();

    if(target) {
        target->notifyMessageProcessed();
    }
}
