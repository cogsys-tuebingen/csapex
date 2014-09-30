/// HEADER
#include <csapex/msg/output.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <iostream>

using namespace csapex;

Output::Output(Settings &settings, const UUID& uuid)
    : Connectable(settings, uuid), force_send_message_(false)
{
}

Output::Output(Settings& settings, Unique* parent, int sub_id)
    : Connectable(settings, parent, sub_id, TYPE_OUT), force_send_message_(false)
{
}

Output::~Output()
{
    BOOST_FOREACH(Input* i, targets_) {
        i->removeConnection(this);
    }
}

int Output::noTargets()
{
    return targets_.size();
}

Output::TargetIterator Output::beginTargets() const
{
    return targets_.begin();
}
Output::TargetIterator Output::endTargets() const
{
    return targets_.end();
}

void Output::removeConnection(Connectable* other_side)
{
    for(std::vector<Input*>::iterator i = targets_.begin(); i != targets_.end();) {
        if(*i == other_side) {
            other_side->removeConnection(this);

            i = targets_.erase(i);

            Q_EMIT connectionRemoved();
            return;

        } else {
            ++i;
        }
    }
}

Command::Ptr Output::removeConnectionCmd(Input* other_side) {
    Command::Ptr removeThis(new command::DeleteConnection(this, other_side));

    return removeThis;
}

Command::Ptr Output::removeAllConnectionsCmd()
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    BOOST_FOREACH(Input* target, targets_) {
        Command::Ptr removeThis(new command::DeleteConnection(this, target));
        removeAll->add(removeThis);
    }

    return removeAll;
}

void Output::removeAllConnectionsNotUndoable()
{
    for(std::vector<Input*>::iterator i = targets_.begin(); i != targets_.end();) {
        (*i)->removeConnection(this);
        i = targets_.erase(i);
    }

    Q_EMIT disconnected(this);
}

void Output::connectForcedWithoutCommand(Input *other_side)
{
    tryConnect(other_side);
}

bool Output::tryConnect(Connectable *other_side)
{
    return connect(other_side);
}

bool Output::connect(Connectable *other_side)
{
    if(!other_side->canInput()) {
        std::cerr << "cannot connect, other side can't input" << std::endl;
        return false;
    }
    if(!other_side->canConnectTo(this, false)) {
        std::cerr << "cannot connect, other side can't connect" << std::endl;
        return false;
    }

    Input* input = dynamic_cast<Input*>(other_side);

    if(!input->acknowledgeConnection(this)) {
        std::cerr << "cannot connect, other side doesn't acknowledge" << std::endl;
        return false;
    }

    apex_assert_hard(input);
    targets_.push_back(input);

    QObject::connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));

    validateConnections();

    return true;
}

bool Output::targetsCanBeMovedTo(Connectable* other_side) const
{
    for(Output::TargetIterator it = beginTargets(); it != endTargets(); ++it) {
        if(!(*it)->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool Output::isConnected() const
{
    return targets_.size() > 0;
}

void Output::connectionMovePreview(Connectable *other_side)
{
    for(Output::TargetIterator it = beginTargets(); it != endTargets(); ++it) {
        Q_EMIT(connectionInProgress(*it, other_side));
    }
}

void Output::validateConnections()
{
    BOOST_FOREACH(Input* target, targets_) {
        target->validateConnections();
    }
}

void Output::publish(ConnectionType::Ptr message)
{
    // update buffer
    message_to_send_ = message;
}

bool Output::hasMessage()
{
    return message_to_send_;
}

void Output::sendMessages()
{
    if(message_to_send_) {
        message_ = message_to_send_;
        message_to_send_.reset();
    } else {
        std::cout << getUUID() << " sends empty message" << std::endl;
        message_ = connection_types::makeEmpty<connection_types::NoMessage>();
    }

    message_->setSequenceNumber(seq_no_);

    // wait for all connected inputs to be able to receive
    //  * inputs can only be connected to this output since they are 1:1
    std::vector<Input*> targets;
    BOOST_FOREACH(Input* i, targets_) {
        if(i->isEnabled()) {
            targets.push_back(i);
        }
    }

    if(!targets.empty()) {
        // all connected inputs are ready to receive, send them the message
        if(targets.size() == 1) {
            targets[0]->inputMessage(message_);
        } else if(targets.size() > 1) {
            BOOST_FOREACH(Input* i, targets) {
                ConnectionType::Ptr msg = message_->clone();
                msg->setSequenceNumber(seq_no_);
                i->inputMessage(msg);
            }
        }
        ++count_;
    }

    ++seq_no_;
    Q_EMIT messageSent(this);
}

void Output::forceSendMessage(bool force)
{
    force_send_message_ = force;
}

ConnectionType::Ptr Output::getMessage()
{
    return message_;
}
