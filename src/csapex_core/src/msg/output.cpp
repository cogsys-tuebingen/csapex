/// HEADER
#include <csapex/msg/output.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>
#include <csapex/profiling/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/debug.h>
#include <csapex/msg/output_transition.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

Output::Output(const UUID& uuid, ConnectableOwnerWeakPtr owner) : Connectable(uuid, owner), transition_(nullptr), state_(State::IDLE)
{
}

Output::~Output()
{
}

void Output::setOutputTransition(OutputTransition* ot)
{
    transition_ = ot;
}

void Output::removeOutputTransition()
{
    transition_ = nullptr;
}

void Output::addConnection(ConnectionPtr connection)
{
    Connectable::addConnection(connection);
}

void Output::removeConnection(Connectable* other_side)
{
    Connectable::removeConnection(other_side);
    if (connections_.empty()) {
        setState(State::IDLE);
    }
}

void Output::notifyMessageProcessed()
{
    // TRACE std::cout << getUUID() << " notified" << std::endl;
    if (isProcessing()) {
        setProcessing(false);
        setState(State::IDLE);
        message_processed(shared_from_this());
    }
}

void Output::notifyMessageProcessed(Connection* connection)
{
    for (auto connection : connections_) {
        if (connection->getState() != Connection::State::DONE) {
            return;
        }
    }

    // TRACE std::cout << getUUID() << " is processed" << std::endl;

    notifyMessageProcessed();
}

void Output::activate()
{
    setState(State::ACTIVE);
}

void Output::setState(State s)
{
    if (state_ == s) {
        return;
    }
    if (s == State::IDLE) {
        // TRACE std::cout << "output " << getUUID() << " is now idle" << std::endl;
    } else {
        // TRACE std::cout << "output " << getUUID() << " is now active" << std::endl;
    }
    state_ = s;
}

Output::State Output::getState() const
{
    return state_;
}

void Output::reset()
{
    Connectable::reset();

    clearBuffer();

    setSequenceNumber(0);
    setState(State::IDLE);
}

std::vector<ConnectionPtr> Output::getConnections() const
{
    return connections_;
}

void Output::removeAllConnectionsNotUndoable()
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    for (std::vector<ConnectionPtr>::iterator i = connections_.begin(); i != connections_.end();) {
        (*i)->to()->removeConnection(this);
        i = connections_.erase(i);
    }

    disconnected(shared_from_this());
}

void Output::enable()
{
    Connectable::enable();
}

void Output::disable()
{
    Connectable::disable();

    if (isProcessing()) {
        for (auto connection : connections_) {
            if (connection->getState() == Connection::State::UNREAD) {
                connection->readToken();
            }
            if (connection->getState() == Connection::State::READ) {
                connection->setTokenProcessed();
            }
        }
    }
}

bool Output::isConnected() const
{
    return !connections_.empty();
}

bool Output::canReceiveToken() const
{
    for (const ConnectionPtr& connection : connections_) {
        if (connection->getState() != Connection::State::NOT_INITIALIZED) {
            return false;
        }
    }
    return true;
}

bool Output::canSendMessages() const
{
    for (const ConnectionPtr& connection : connections_) {
        if (connection->getState() == Connection::State::NOT_INITIALIZED) {
            return false;
        }
    }
    return !isProcessing();
}

void Output::publish()
{
    apex_assert_hard(isEnabled());
    auto msg = getToken();
    apex_assert_hard(msg);

    setProcessing(true);

    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    bool sent = false;
    for (auto connection : connections_) {
        if (connection->isEnabled()) {
            connection->setToken(msg);
            sent = true;
        }
    }

    if (!sent) {
        notifyMessageProcessed();
    }
}

void Output::republish()
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    if (isProcessing()) {
        auto msg = getToken();
        apex_assert_hard(msg);

        for (auto connection : connections_) {
            if (connection->isEnabled() && connection->getState() == Connection::State::DONE) {
                connection->setToken(msg);
            }
        }
    }
}

void Output::addStatusInformation(std::stringstream& status_stream) const
{
    status_stream << ", state: ";
    switch (getState()) {
        case Output::State::ACTIVE:
            status_stream << "ACTIVE";
            break;
        case Output::State::IDLE:
            status_stream << "IDLE";
            break;
        default:
            status_stream << "UNKNOWN";
    }
}
