/// HEADER
#include <csapex/model/connector_remote.h>

/// PROJECT
#include <csapex/io/protcol/connector_requests.h>

using namespace csapex;


ConnectorRemote::ConnectorRemote(UUID uuid, ConnectableOwnerPtr owner,
                                 SessionPtr session,
                                 ConnectorPtr tmp_connector)
    : Connector(uuid, owner),
      Remote(session),
      tmp_connector_(tmp_connector),
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
    channel_ = session->openChannel(uuid.getAbsoluteUUID());
//    continue: relay these signals generically with the channel

//    when a signal is received, update the cache values, then reemit.

    observe(tmp_connector_->enabled_changed, enabled_changed);

    observe(tmp_connector_->essential_changed, essential_changed);

    observe(tmp_connector_->disconnected, disconnected);
    observe(tmp_connector_->connectionStart, connectionStart);

    observe(tmp_connector_->connection_added_to, connection_added_to);
    observe(tmp_connector_->connection_removed_to, connection_removed_to);

    observe(tmp_connector_->connection_added, connection_added);
    observe(tmp_connector_->connection_faded, connection_faded);

    observe(tmp_connector_->connectionEnabled, connectionEnabled);
    observe(tmp_connector_->message_processed, message_processed);
    observe(tmp_connector_->connectableError, connectableError);

    observe(tmp_connector_->typeChanged, typeChanged);
    observe(tmp_connector_->labelChanged, labelChanged);

    /**
     * begin: connect signals
     **/
    #define HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
    observe(tmp_connector_->signal, [this](const type& new_value) { \
        value_##function##_ = new_value;\
        signal(new_value); \
    });
    #include <csapex/model/connector_remote_accessors.hpp>
    /**
     * end: connect signals
     **/
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
