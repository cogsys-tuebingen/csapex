/// HEADER
#include <csapex/model/connector_remote.h>

/// PROJECT
#include <csapex/io/protcol/connector_requests.h>
#include <csapex/io/protcol/connector_notes.h>
#include <csapex/io/channel.h>

/// SYSTEM
#include <iostream>

using namespace csapex;


ConnectorRemote::ConnectorRemote(const SessionPtr& session, UUID uuid, ConnectableOwnerPtr owner)
    : Connector(uuid, owner),
      Remote(session),
      /**
       * begin: initialize caches
       **/
      #define HANDLE_ACCESSOR(_enum, type, function)
      #define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
          has_##function##_(false),
      #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
          has_##function##_(false),
      #include <csapex/model/connector_remote_accessors.hpp>
      /**
       * end: initialize caches
       **/
      connected_(false)
{
    channel_ = session_->openChannel(uuid.getAbsoluteUUID());

    observe(channel_->note_received, [this](const io::NoteConstPtr& note){
        if(const std::shared_ptr<ConnectorNote const>& cn = std::dynamic_pointer_cast<ConnectorNote const>(note)) {
            switch(cn->getNoteType()) {
                /**
                 * begin: connect signals
                 **/
                #define HANDLE_ACCESSOR(_enum, type, function)
                #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
                #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
                    case ConnectorNoteType::function##Changed: \
                    { \
                        auto new_value = boost::any_cast<type>(cn->getPayload()); \
                        value_##function##_ = new_value;\
                        signal(new_value); \
                    } \
                    break;
                #include <csapex/model/connector_remote_accessors.hpp>
                /**
                 * end: connect signals
                 **/
            }
        }

    });
}


ConnectorRemote::ConnectorRemote(const SessionPtr& session, UUID uuid, ConnectableOwnerPtr owner, const ConnectorDescription& cd)
    : ConnectorRemote(session, uuid, owner)
{
    cache_getDescription_ = cd;
    has_getDescription_ = true;
}

bool ConnectorRemote::isConnectedTo(const UUID &other) const
{
    return request<bool, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::IsConnectedTo, getUUID().getAbsoluteUUID(), other);
}

bool ConnectorRemote::isActivelyConnectedTo(const UUID &other) const
{
    return request<bool, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::IsActivelyConnectedTo, getUUID().getAbsoluteUUID(), other);
}

/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function) \
type ConnectorRemote::function() const\
{\
    return request<type, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::_enum, getUUID().getAbsoluteUUID());\
}
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
type ConnectorRemote::function() const\
{\
    if(!has_##function##_) { \
        cache_##function##_ = request<type, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::_enum, getUUID().getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return cache_##function##_; \
}
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
type ConnectorRemote::function() const\
{\
    if(!has_##function##_) { \
        value_##function##_ = request<type, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::_enum, getUUID().getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return value_##function##_; \
}

#include <csapex/model/connector_remote_accessors.hpp>
/**
 * end: generate getters
 **/
