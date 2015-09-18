/// HEADER
#include <csapex/msg/input_transition.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <sstream>

using namespace csapex;

InputTransition::InputTransition(NodeWorker* node)
    : Transition(node), one_input_is_dynamic_(false)
{
}

void InputTransition::addInput(InputPtr input)
{
    auto ca = input->connection_added.connect([this](ConnectionPtr connection) {
        addConnection(connection);
    });
    input_signal_connections_[input].push_back(ca);

    auto cf = input->connection_faded.connect([this](ConnectionPtr connection) {
        fadeConnection(connection);
    });
    input_signal_connections_[input].push_back(cf);
}

void InputTransition::removeInput(InputPtr input)
{
    for(auto f : input_signal_connections_[input]) {
        f.disconnect();
    }
    input_signal_connections_.erase(input);
}

void InputTransition::establishConnections()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    auto unestablished_connections = unestablished_connections_;
    lock.unlock();

    if(!unestablished_connections.empty()) {
        for(auto c : unestablished_connections) {
            if(!c->isSinkEstablished()) {
                c->establishSink();
            }
            if(c->isSourceEstablished() && c->isSinkEstablished()) {
                establishConnection(c);
            }
        }
    }
}

void InputTransition::connectionRemoved(Connection *connection)
{
    Transition::connectionRemoved(connection);
    connection->fadeSink();
}

void InputTransition::reset()
{
    for(auto input : node_->getAllInputs()) {
        input->reset();
    }
    for(ConnectionPtr connection : established_connections_) {
        connection->reset();
    }
}

void InputTransition::connectionAdded(Connection *connection)
{
    Transition::connectionAdded(connection);

    trackConnection(connection, connection->new_message.connect([this]() {
        //        std::cerr << "new message in " << node_->getUUID() << std::endl;
        //        update();

        // TODO: is this necessary?
        node_->triggerCheckTransitions();
    }));

    trackConnection(connection, connection->endpoint_established.connect([this]() {
        // establish();
        node_->triggerCheckTransitions();
    }));

    one_input_is_dynamic_ = false;
    for(auto input : node_->getAllInputs()) {
        if(input->isDynamic()) {
            one_input_is_dynamic_ = true;
            break;
        }
    }
}

void InputTransition::fireIfPossible()
{
    if(!node_->canProcess()) {
        return;
    }

    // TODO: replace tick with process where possible
    if(node_->isSource() && !node_->isSink()) {
        //        apex_assert_hard(established_connections_.empty());
        //fire(); -> instead of tick!!!!
    } else {
        if(isEnabled()) {
            apex_assert_hard(areAllConnections(Connection::State::READ, Connection::State::UNREAD));
            if(node_->isEnabled()) {

                for(const auto& connection : established_connections_) {
                    if(connection->getState() == Connection::State::NOT_INITIALIZED) {
                        if(connection->isEstablished()) {
                            return;
                        }
                    }
                }


                int highest_deviant_seq = findHighestDeviantSequenceNumber();

                if(highest_deviant_seq >= 0) {
                    notifyOlderConnections(highest_deviant_seq);

                } else {
                    fire();
                }
            }
        }
    }
}

bool InputTransition::isEnabled() const
{
    if(isOneConnection(Connection::State::READY_TO_RECEIVE)) {
        return false;
    }
    if(!areAllConnections(Connection::State::UNREAD, Connection::State::READ/*, Connection::State::DONE*/)) {
        return false;
    }
    return !areAllConnections(Connection::State::READ);
}

int InputTransition::findHighestDeviantSequenceNumber() const
{
    int highest_deviant_seq = -1;

    bool a_connection_deviates = false;
    for(auto input : node_->getAllInputs()) {
        //        std::cerr << "input message from " <<  node_->getUUID() << " -> " << input->getUUID() << std::endl;

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
    for(auto input : node_->getAllInputs()) {
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
}

void InputTransition::notifyMessageProcessed()
{
    apex_assert_hard(areAllConnections(Connection::State::READ));

    bool has_multipart = false;
    bool multipart_are_done = true;

    for(auto& c : established_connections_) {
        int f = c->getMessage()->flags.data;
        if(f & (int) ConnectionType::Flags::Fields::MULTI_PART) {
            has_multipart = true;
            bool last_part = f & (int) ConnectionType::Flags::Fields::LAST_PART;
            multipart_are_done &= last_part;
        }
    }

    if(has_multipart && !multipart_are_done) {
        for(ConnectionPtr& c : established_connections_) {
            int f = c->getMessage()->flags.data;

            if(f & (int) ConnectionType::Flags::Fields::MULTI_PART) {
                //                c->setState(Connection::State::DONE);
                c->setMessageProcessed();
            }
        }

    } else {
        apex_assert_hard(areAllConnections(Connection::State::READ));

        for(ConnectionPtr& c : established_connections_) {
            c->setMessageProcessed();
        }

        if(hasFadingConnection()) {
            removeFadingConnections();
        }

    }
}

void InputTransition::fire()
{
    apex_assert_hard(node_->canProcess());
    apex_assert_hard(node_->isEnabled());
    apex_assert_hard(node_->isProcessingEnabled());
    apex_assert_hard(!isOneConnection(Connection::State::DONE));
    //    apex_assert_hard(!isConnection(Connection::State::NOT_INITIALIZED));
    apex_assert_hard(!isOneConnection(Connection::State::READY_TO_RECEIVE));
    apex_assert_hard(areAllConnections(Connection::State::UNREAD, Connection::State::READ));
    apex_assert_hard(established_connections_.empty() || !areAllConnections(Connection::State::READ));

    std::vector<DynamicInput*> dynamic_inputs;
    ConnectionTypeConstPtr dynamic_message_part;
    ConnectionPtr dynamic_connection;

    for(auto input : node_->getAllInputs()) {
        if(input->isDynamic() && input->isConnected()) {
            auto connections = input->getConnections();
            apex_assert_hard(connections.size() == 1);
            dynamic_connection = connections.front();
            auto s = dynamic_connection->getState();
            apex_assert_hard(s == Connection::State::READ ||
                             s == Connection::State::UNREAD);
            dynamic_message_part = dynamic_connection->getMessage();
            apex_assert_hard(dynamic_message_part != nullptr);
            DynamicInput* di = dynamic_cast<DynamicInput*>(input.get());
            dynamic_inputs.push_back(di);
        }
    }

    apex_assert_hard(dynamic_inputs.size() <= 1);
    if(!dynamic_inputs.empty()) {
        DynamicInput* di = dynamic_inputs.front();
        bool every_part_received = di->inputMessagePart(dynamic_message_part);
        if(!every_part_received) {
            // wait until everything is here, then proceed.
            auto s = dynamic_connection->getState();
            apex_assert_hard(s == Connection::State::UNREAD ||
                             s == Connection::State::READ);
            dynamic_connection->setState(Connection::State::READ);
            //            dynamic_connection->setState(Connection::State::DONE);
            dynamic_connection->setMessageProcessed();
            return;
        }
    }

    //    std::cerr << "fire " <<  node_->getUUID() << std::endl;

    for(auto input : node_->getAllInputs()) {
        //        std::cerr << "input message from " <<  node_->getUUID() << " -> " << input->getUUID() << std::endl;

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

    for(auto& c : established_connections_) {
        auto s = c->getState();
        apex_assert_hard(c->isEstablished());
        apex_assert_hard(s != Connection::State::NOT_INITIALIZED);
        apex_assert_hard(s != Connection::State::READY_TO_RECEIVE);
        apex_assert_hard(s == Connection::State::UNREAD ||
                         s == Connection::State::READ);
    }

    apex_assert_hard(!areAllConnections(Connection::State::DONE));
    for(auto& c : established_connections_) {
        c->setState(Connection::State::READ);
    }

    //    std::cerr << "-> process " <<  node_->getUUID() << std::endl;

    apex_assert_hard(node_->isEnabled());
    node_->triggerProcess();
}
