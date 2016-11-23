/// HEADER
#include <csapex/msg/input_transition.h>

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/no_message.h>
#include <csapex/utility/debug.h>

/// SYSTEM
#include <sstream>
#include <iostream>

using namespace csapex;

InputTransition::InputTransition(delegate::Delegate0<> activation_fn)
    : Transition(activation_fn), forwarded_(false), processed_(false)
{
}

InputTransition::InputTransition()
    : Transition(), forwarded_(false), processed_(false)
{
}

InputPtr InputTransition::getInput(const UUID& id) const
{
    return inputs_.at(id);
}

std::vector<UUID> InputTransition::getInputs() const
{
    std::vector<UUID> res;
    for(const auto& pair : inputs_) {
        res.push_back(pair.second->getUUID());
    }
    std::sort(res.begin(), res.end());
    return res;
}

void InputTransition::addInput(InputPtr input)
{
    input->setInputTransition(this);

    // remember the input
    inputs_[input->getUUID()] = input;

    // connect signals
    auto cm = input->message_available.connect([this](Connection*) {
            checkIfEnabled();
});
    input_signal_connections_[input].push_back(cm);

    auto ca = input->connection_added.connect([this](ConnectionPtr connection) {
            addConnection(connection);
});
    input_signal_connections_[input].push_back(ca);

    auto cf = input->connection_faded.connect([this](ConnectionPtr connection) {
            removeConnection(connection);
});
    input_signal_connections_[input].push_back(cf);
}

void InputTransition::removeInput(InputPtr input)
{
    input->removeInputTransition();

    // disconnect signals
    for(auto f : input_signal_connections_[input]) {
        f.disconnect();
    }
    input_signal_connections_.erase(input);

    // forget the input
    inputs_.erase(input->getUUID());
}

void InputTransition::connectionRemoved(Connection *connection)
{
    Transition::connectionRemoved(connection);
}

void InputTransition::reset()
{
    for(auto pair : inputs_) {
        InputPtr input = pair.second;
        input->reset();
    }
    for(ConnectionPtr connection : connections_) {
        connection->reset();
    }

    forwarded_ = false;
    processed_ = false;

    Transition::reset();
}

void InputTransition::connectionAdded(Connection *connection)
{
    Transition::connectionAdded(connection);

    bool needs_message = isOneConnection(Connection::State::UNREAD) && connection->getState() == Connection::State::NOT_INITIALIZED;
    if(needs_message) {
        connection->setToken(Token::makeEmpty<connection_types::NoMessage>());
    }

    updateConnections();
}

bool InputTransition::isEnabled() const
{
    if(forwarded_) {
        APEX_DEBUG_CERR <<"not enabled because already forwarded" << std::endl;
        return false;
    }

    if(!areAllConnections(Connection::State::UNREAD, Connection::State::READ/*, Connection::State::DONE*/)) {
        APEX_DEBUG_CERR <<"not enabled because not all connections read or unread" << std::endl;
        return false;
    }

    // TODO: is this necessary?
    for(const ConnectionPtr& connection : connections_) {
        if(connection->isEnabled() && connection->getState() == Connection::State::NOT_INITIALIZED) {
            APEX_DEBUG_CERR <<"not enabled because a connection is not initialized" << std::endl;
            return false;
        }
    }

    if(!areAllConnections(Connection::State::READ)) {
        return true;
    } else {
        APEX_DEBUG_CERR <<"not enabled because a connection is read" << std::endl;
        return false;
    }
}

int InputTransition::findHighestDeviantSequenceNumber() const
{
    int highest_deviant_seq = -1;

    bool a_connection_deviates = false;
    for(auto pair : inputs_) {
        InputPtr input = pair.second;

        if(input->isConnected()) {
            auto connections = input->getConnections();
            apex_assert_hard(connections.size() == 1);
            ConnectionPtr connection = connections.front();
            TokenConstPtr token = connection->getToken();

            int s = token->getSequenceNumber();
            if(highest_deviant_seq != -1 && highest_deviant_seq != s) {
                a_connection_deviates = true;
            }
            if(s > highest_deviant_seq) {
                highest_deviant_seq = s;
            }
        }
    }
    if(a_connection_deviates) {
        return highest_deviant_seq;
    } else {
        return -1;
    }
}

void InputTransition::notifyMessageRead()
{
    if(!forwarded_) {
        return;
    }
    //    apex_assert_hard(areAllConnections(Connection::State::READ, Connection::State::NOT_INITIALIZED));

    //    for(ConnectionPtr& c : connections_) {
    //        c->setTokenRead();
    //    }
    //    forwarded_ = false;
}

void InputTransition::notifyMessageProcessed()
{
    if(processed_) {
        APEX_DEBUG_CERR <<"input transition not notified, is processed" << std::endl;
        return;
    }

    if(areAllConnections(Connection::State::READ, Connection::State::NOT_INITIALIZED)) {
        APEX_DEBUG_CERR <<"input transition notified" << std::endl;
        forwarded_ = false;
        processed_ = true;
        for(ConnectionPtr& c : connections_) {
            c->setTokenProcessed();
        }

    } else {
        APEX_DEBUG_CERR <<"input transition not notified, some connection is not read" << std::endl;
    }

}

bool InputTransition::areMessagesComplete() const
{
    for(auto pair : inputs_) {
        InputPtr input = pair.second;
        if(input->isConnected() && !input->hasReceived()) {
            return false;
        }
    }
    return true;
}

void InputTransition::forwardMessages()
{
    processed_ = false;

    if(hasConnection()) {
        apex_assert_hard(!forwarded_);

        apex_assert_hard(!isOneConnection(Connection::State::DONE));
        apex_assert_hard(areAllConnections(Connection::State::UNREAD, Connection::State::READ));


        updateConnections();
        apex_assert_hard(connections_.empty() || !areAllConnections(Connection::State::READ));

        for(auto pair : inputs_) {
            InputPtr input = pair.second;

            if(input->isConnected()) {
                auto connections = input->getConnections();
                apex_assert_hard(connections.size() == 1);
                ConnectionPtr connection = connections.front();
                auto s = connection->getState();
                apex_assert_hard(s == Connection::State::READ ||
                                 s == Connection::State::UNREAD);
                TokenPtr token = connection->getToken();
                apex_assert_hard(token != nullptr);
                input->setToken(token);
            } else {
                input->setToken(Token::makeEmpty<connection_types::NoMessage>());
            }
        }

        for(auto& c : connections_) {
            auto s = c->getState();
            //        apex_assert_hard(c->isEstablished());
            apex_assert_hard(s != Connection::State::NOT_INITIALIZED);
            apex_assert_hard(s == Connection::State::UNREAD ||
                             s == Connection::State::READ);
        }

        apex_assert_hard(!areAllConnections(Connection::State::DONE));
        for(auto& c : connections_) {
            c->setState(Connection::State::READ);
        }
    }

    forwarded_ = true;
}

bool InputTransition::areMessagesProcessed() const
{
    return processed_;
}

bool InputTransition::areMessagesForwarded() const
{
    return forwarded_;
}
