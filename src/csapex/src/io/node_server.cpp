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
#include <csapex/io/protcol/profiler_note.h>
#include <csapex/profiling/profiler.h>

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

    observe(node->interval_start, [this, channel](NodeFacade* facade, ActivityType type, std::shared_ptr<const Interval> stamp){
        channel->sendNote<NodeNote>(NodeNoteType::IntervalStartTriggered, type, stamp);
    });

    observe(node->interval_end, [this, channel](NodeFacade* facade, std::shared_ptr<const Interval> stamp){
        channel->sendNote<NodeNote>(NodeNoteType::IntervalEndTriggered, stamp);
    });
    observe(node->error_event, [this, channel](bool e, const std::string& msg, ErrorState::ErrorLevel level){
        channel->sendNote<NodeNote>(NodeNoteType::ErrorEvent, e, msg, level);
    });
    observe(node->notification, [this, channel](Notification n){
        channel->sendNote<NodeNote>(NodeNoteType::Notification, n);
    });


    observe(node->start_profiling, [this, channel](NodeFacade*){
        channel->sendNote<NodeNote>(NodeNoteType::ProfilingStartTriggered);
    });
    observe(node->stop_profiling, [this, channel](NodeFacade*){
        channel->sendNote<NodeNote>(NodeNoteType::ProfilingStopTriggered);
    });


    ProfilerPtr profiler = node->getProfiler();
    observe(profiler->enabled_changed, [this, channel](bool enabled){
        channel->sendNote<ProfilerNote>(ProfilerNoteType::EnabledChanged, enabled);
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
