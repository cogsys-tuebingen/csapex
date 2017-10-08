/// HEADER
#include <csapex/io/protcol/connector_requests.h>

/// PROJECT
#include <csapex/command/command.h>
#include <csapex/io/feedback.h>
#include <csapex/io/raw_message.h>
#include <csapex/io/session.h>
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(ConnectorRequests)

using namespace csapex;

///
/// REQUEST
///
ConnectorRequests::ConnectorRequest::ConnectorRequest(ConnectorRequestType request_type, const AUUID &uuid)
    : RequestImplementation(0),
      request_type_(request_type),
      uuid_(uuid)
{
    apex_assert(uuid_ != UUID::NONE);
}

ConnectorRequests::ConnectorRequest::ConnectorRequest(ConnectorRequestType request_type, const AUUID &uuid, const boost::any &payload)
    : RequestImplementation(0),
      request_type_(request_type),
      uuid_(uuid),
      payload_(payload)
{
    apex_assert(uuid_ != UUID::NONE);
}

ConnectorRequests::ConnectorRequest::ConnectorRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}


ResponsePtr ConnectorRequests::ConnectorRequest::execute(const SessionPtr &session, CsApexCore &core) const
{
    GraphFacadeLocalPtr gf = core.getRoot();

    ConnectorPtr c = gf->findConnectorNoThrow(uuid_);
    if(!c) {
        return std::make_shared<Feedback>(std::string("unknown connector id ") + uuid_.getFullName(),
                                          getRequestID());
    }

    switch(request_type_)
    {
    case ConnectorRequests::ConnectorRequestType::IsConnectedTo:
    {
        UUID other_id = boost::any_cast<UUID>(payload_);
        return std::make_shared<ConnectorResponse>(request_type_, c->isConnectedTo(other_id), getRequestID(), uuid_);
    }
    case ConnectorRequests::ConnectorRequestType::IsActivelyConnectedTo:
    {
        UUID other_id = boost::any_cast<UUID>(payload_);
        return std::make_shared<ConnectorResponse>(request_type_, c->isActivelyConnectedTo(other_id), getRequestID(), uuid_);
    }

        /**
         * begin: generate cases
         **/
#define HANDLE_ACCESSOR(_enum, type, function) \
    case ConnectorRequests::ConnectorRequestType::_enum:\
        return std::make_shared<ConnectorResponse>(request_type_, c->function(), getRequestID(), uuid_);
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)

    #include <csapex/model/connector_remote_accessors.hpp>
        /**
         * end: generate cases
         **/

    default:
        return std::make_shared<Feedback>(std::string("unknown node request type ") + std::to_string((int)request_type_),
                                          getRequestID());
    }

    return std::make_shared<ConnectorResponse>(request_type_, getRequestID(), uuid_);
}

void ConnectorRequests::ConnectorRequest::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << uuid_;
    data << payload_;
}

void ConnectorRequests::ConnectorRequest::deserialize(SerializationBuffer& data)
{
    data >> request_type_;
    data >> uuid_;
    data >> payload_;
}

///
/// RESPONSE
///

ConnectorRequests::ConnectorResponse::ConnectorResponse(ConnectorRequestType request_type, uint8_t request_id, const AUUID& uuid)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      uuid_(uuid)
{

}
ConnectorRequests::ConnectorResponse::ConnectorResponse(ConnectorRequestType request_type, boost::any result, uint8_t request_id, const AUUID& uuid)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      uuid_(uuid),
      result_(result)
{

}

ConnectorRequests::ConnectorResponse::ConnectorResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void ConnectorRequests::ConnectorResponse::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << uuid_;
    data << result_;
}

void ConnectorRequests::ConnectorResponse::deserialize(SerializationBuffer& data)
{
    data >> request_type_;
    data >> uuid_;
    data >> result_;
}
