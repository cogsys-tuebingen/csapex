/// HEADER
#include <csapex/io/node_server.h>

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/io/session.h>
#include <csapex/io/connector_server.h>
#include <csapex/io/channel.h>
#include <csapex/io/protcol/node_notes.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeServer::NodeServer(SessionPtr session)
    : session_(session)
{
    connector_server_ = std::make_shared<ConnectorServer>(session_);
}

NodeServer::~NodeServer()
{
}


void NodeServer::startObserving(const NodeFacadeLocalPtr &node)
{
    io::ChannelPtr channel = session_->openChannel(node->getAUUID());

    for(const ConnectorDescription& cd : node->getExternalConnectors()) {
        ConnectorPtr c = node->getConnector(cd.id);
        connector_server_->startObserving(std::dynamic_pointer_cast<Connectable>(c));
    }


    /**
     * begin: connect signals
     **/
    #define HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
    observe(node->signal, [this, channel](const type& new_value){ \
        channel->sendNote<NodeNote>(NodeNoteType::function##Changed, new_value);  \
    });
    #define HANDLE_SIGNAL(_enum, signal) \
    observe(node->signal, [this, channel](){ \
        channel->sendNote<NodeNote>(NodeNoteType::_enum##Triggered);  \
    });

    #include <csapex/model/node_facade_remote_accessors.hpp>
    /**
     * end: connect signals
     **/


    observe(node->connector_created, [this, channel](ConnectorDescription c){
        channel->sendNote<NodeNote>(NodeNoteType::ConnectorCreatedTriggered, c);
    });

    observe(node->connector_removed, [this, channel](ConnectorDescription c){
        channel->sendNote<NodeNote>(NodeNoteType::ConnectorRemovedTriggered, c);
    });

    observe(node->connection_start, [this, channel](ConnectorDescription c){
        channel->sendNote<NodeNote>(NodeNoteType::ConnectionStartTriggered, c);
    });

    observe(node->connection_done, [this, channel](ConnectorDescription c){
        channel->sendNote<NodeNote>(NodeNoteType::ConnectionDoneTriggered, c);
    });

    channels_[node->getAUUID()] = channel;
}

void NodeServer::stopObserving(const NodeFacadeLocalPtr &node)
{
    std::cerr << "stop serving node: " << node->getAUUID() << std::endl;

    auto pos = channels_.find(node->getAUUID());
    if(pos != channels_.end()) {
        channels_.erase(pos);
    }
}
