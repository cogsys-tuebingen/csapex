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
    std::unique_lock<std::recursive_mutex> processing_lock(processing_mutex_);

    // Multiple connections can send this signal concurrently.
    // The first of them can be handled *after* all connections are already done, so
    // we need to make sure to only forward this once to the owner

    if (isProcessing()) {
        // std::cerr << getUUID() << " :::: "
        //           << "-> notifyMessageProcesed " << std::endl;
        setProcessing(false);
        setState(State::IDLE);

        processing_lock.unlock();

        for (const ConnectionPtr& connection : connections_) {
            apex_assert_hard(connection->getState() == Connection::State::DONE);
        }
        message_processed(shared_from_this());
    } else {
        // std::cerr << getUUID() << " :::: "
        //           << "-> cannot notifyMessageProcesed, not processing " << std::endl;
    }
}

void Output::notifyMessageProcessed(Connection* connection)
{
//     std::cerr << getUUID() << " :::: " << *connection << "-> sent notifyProcessed " << std::endl;
//     for (auto connection : connections_) {
//         std::string s;
//         switch (connection->getState()) {
//             case Connection::State::DONE:
//                 s = "DONE / NOT_INITIALIZED";
//                 break;
//             case Connection::State::UNREAD:
//                 s = "UNREAD";
//                 break;
//             case Connection::State::READ:
//                 s = "READ";
//                 break;
//         }
//         std::cerr << getUUID() << " :::: " << *connection << "-> state: " << s << std::endl;
//     }

    for (auto connection : connections_) {
        if (connection->getState() != Connection::State::DONE) {
            // std::cerr << getUUID() << " :::: " << *connection << "-> is not yet done " << std::endl;
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
    std::unique_lock<std::recursive_mutex> processing_lock(processing_mutex_);
    apex_assert_hard(!isProcessing());
    apex_assert_hard(isEnabled());
    auto msg = getToken();
    apex_assert_hard(msg);

    setProcessing(true);

    processing_lock.unlock();

    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    bool sent = false;
    for (auto connection : connections_) {
        if (connection->isEnabled()) {
            // std::cerr << getUUID() << " :::: " << *connection << "-> set token to: " << msg->getTokenData()->descriptiveName() << std::endl;
            connection->setToken(msg, true);
            sent = true;
        // } else {
        //     std::cerr << getUUID() << " :::: " << *connection << "-> set no token, connection is not enabled" << std::endl;
        }
    }

    for (auto connection : connections_) {
        if (connection->isEnabled()) {
            connection->notifyMessageSet();
        }
    }

    if (!sent) {
        // std::cerr << getUUID() << " :::: "
        //           << "is notified processed in publish because no message is sent" << std::endl;
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
