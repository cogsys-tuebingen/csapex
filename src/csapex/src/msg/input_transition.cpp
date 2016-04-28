/// HEADER
#include <csapex/msg/input_transition.h>

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/no_message.h>

/// SYSTEM
#include <sstream>
#include <iostream>

using namespace csapex;

InputTransition::InputTransition(delegate::Delegate0<> activation_fn)
    : Transition(activation_fn), one_input_is_dynamic_(false), forwarded_(false)
{
}

InputTransition::InputTransition()
    : Transition(), one_input_is_dynamic_(false), forwarded_(false)
{
}

InputPtr InputTransition::getInput(const UUID& id) const
{
    return inputs_.at(id);
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

    Transition::reset();
}

void InputTransition::connectionAdded(Connection *connection)
{
    Transition::connectionAdded(connection);

    one_input_is_dynamic_ = false;
    for(auto pair : inputs_) {
        InputPtr input = pair.second;
        if(input->isDynamic()) {
            one_input_is_dynamic_ = true;
            break;
        }
    }

    updateConnections();
}

bool InputTransition::isEnabled() const
{
    if(forwarded_) {
        return false;
    }

    if(!areAllConnections(Connection::State::UNREAD, Connection::State::READ/*, Connection::State::DONE*/)) {
        return false;
    }

    // TODO: is this necessary?
    for(const auto& connection : connections_) {
        if(connection->isEnabled() && connection->getState() == Connection::State::NOT_INITIALIZED) {
            return false;
        }
    }

    return !areAllConnections(Connection::State::READ);
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
            auto connection = connections.front();
            auto msg = connection->getMessage();

            int s = msg->sequenceNumber();
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

void InputTransition::notifyOlderConnections(int seq)
{
    for(auto pair : inputs_) {
        InputPtr input = pair.second;
        if(input->isConnected()) {
            auto connections = input->getConnections();
            apex_assert_hard(connections.size() == 1);
            ConnectionPtr connection = connections.front();
            auto msg = connection->getMessage();

            int s = msg->sequenceNumber();
            if(seq != s) {
                std::cout << input->getUUID().getFullName() << " has seq " << s << " instead of " << seq << std::endl;
                connection->setState(Connection::State::READ);
                connection->setMessageProcessed();
            }
        }
    }
    for(auto pair : inputs_) {
        InputPtr input = pair.second;
        if(input->isConnected()) {
            auto connections = input->getConnections();
            apex_assert_hard(connections.size() == 1);
            ConnectionPtr connection = connections.front();
            auto msg = connection->getMessage();

            int s = msg->sequenceNumber();
            if(seq == s) {
                std::cout << input->getUUID().getFullName() << " has seq " << s << std::endl;
            }
        }
    }
}
void InputTransition::notifyMessageProcessed()
{
    if(!forwarded_) {
        return;
    }
    apex_assert_hard(areAllConnections(Connection::State::READ, Connection::State::NOT_INITIALIZED));

    bool has_multipart = false;
    bool multipart_are_done = true;

    for(auto& c : connections_) {
        if(c->getMessage()) {
            int f = c->getMessage()->flags.data;
            if(f & (int) Token::Flags::Fields::MULTI_PART) {
                has_multipart = true;
                bool last_part = f & (int) Token::Flags::Fields::LAST_PART;
                multipart_are_done &= last_part;
            }
        }
    }

    if(has_multipart && !multipart_are_done) {
        for(ConnectionPtr& c : connections_) {
            if(c->getMessage()) {
                int f = c->getMessage()->flags.data;

                if(f & (int) Token::Flags::Fields::MULTI_PART) {
                    //                c->setState(Connection::State::DONE);
                    c->setMessageProcessed();
                }
            }
        }

    } else {
        apex_assert_hard(areAllConnections(Connection::State::READ, Connection::State::NOT_INITIALIZED));

        for(ConnectionPtr& c : connections_) {
            c->setMessageProcessed();
        }
    }
    apex_assert_hard(areAllConnections(Connection::State::DONE));
    forwarded_ = false;
}

void InputTransition::forwardMessages()
{
    apex_assert_hard(!forwarded_);

    apex_assert_hard(!isOneConnection(Connection::State::DONE));
    apex_assert_hard(areAllConnections(Connection::State::UNREAD, Connection::State::READ));

    updateConnections();
    apex_assert_hard(connections_.empty() || !areAllConnections(Connection::State::READ));

    for(auto pair : inputs_) {
        InputPtr input = pair.second;

        if(input->isConnected()) {
            if(input->isDynamic()) {
                DynamicInput* di = dynamic_cast<DynamicInput*>(input.get());
                di->composeMessage();

            } else {
                auto connections = input->getConnections();
                apex_assert_hard(connections.size() == 1);
                auto connection = connections.front();
                auto s = connection->getState();
                apex_assert_hard(s == Connection::State::READ ||
                                 s == Connection::State::UNREAD);
                auto msg = connection->getMessage();
                apex_assert_hard(msg != nullptr);
                input->inputMessage(msg);
            }
        } else {
            input->inputMessage(connection_types::makeEmpty<connection_types::NoMessage>());
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

    forwarded_ = true;
}
