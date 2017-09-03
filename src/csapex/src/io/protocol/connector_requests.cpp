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
    case ConnectorRequests::ConnectorRequestType::GetType:
        return std::make_shared<NodeResponse>(request_type_, *c->getType(), getRequestID(), uuid_);

#define GENERATE_GETTER(type, function, _enum) \
    case ConnectorRequests::ConnectorRequestType::_enum:\
        return std::make_shared<NodeResponse>(request_type_, c->function(), getRequestID(), uuid_);

    GENERATE_GETTER(int, getCount, GetCount)
    GENERATE_GETTER(bool, canOutput, CanOutput)
    GENERATE_GETTER(bool, canInput, CanInput)
    GENERATE_GETTER(bool, isOutput, IsOutput)
    GENERATE_GETTER(bool, isInput, IsInput)
    GENERATE_GETTER(bool, isOptional, IsOptional)
    GENERATE_GETTER(bool, isSynchronous, IsSynchronous)
    GENERATE_GETTER(bool, isVirtual, IsVirtual)
    GENERATE_GETTER(bool, isParameter, IsParameter)
    GENERATE_GETTER(bool, isGraphPort, IsGraphPort)
    GENERATE_GETTER(bool, isEssential, IsEssential)
    GENERATE_GETTER(std::string, getLabel, GetLabel)
    GENERATE_GETTER(ConnectorType, getConnectorType, GetConnectorType)
    GENERATE_GETTER(ConnectorDescription, getDescription, GetDescription)
    GENERATE_GETTER(bool, isEnabled, IsEnabled)
    GENERATE_GETTER(int, sequenceNumber, GetSequenceNumber)
    GENERATE_GETTER(int, countConnections, GetConnectionCount)
    GENERATE_GETTER(bool, hasActiveConnection, HasActiveConnection)
    GENERATE_GETTER(bool, isConnected, IsConnected)
    GENERATE_GETTER(std::string, makeStatusString, MakeStatusString)


    default:
        return std::make_shared<Feedback>(std::string("unknown node request type ") + std::to_string((int)request_type_),
                                          getRequestID());
    }

    return std::make_shared<NodeResponse>(request_type_, getRequestID(), uuid_);
}

void ConnectorRequests::ConnectorRequest::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << uuid_;
}

void ConnectorRequests::ConnectorRequest::deserialize(SerializationBuffer& data)
{
    data >> request_type_;
    data >> uuid_;
}

///
/// RESPONSE
///

ConnectorRequests::NodeResponse::NodeResponse(ConnectorRequestType request_type, uint8_t request_id, const AUUID& uuid)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      uuid_(uuid)
{

}
ConnectorRequests::NodeResponse::NodeResponse(ConnectorRequestType request_type, boost::any result, uint8_t request_id, const AUUID& uuid)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      uuid_(uuid),
      result_(result)
{

}

ConnectorRequests::NodeResponse::NodeResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void ConnectorRequests::NodeResponse::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << uuid_;
    data << result_;
}

void ConnectorRequests::NodeResponse::deserialize(SerializationBuffer& data)
{
    data >> request_type_;
    data >> uuid_;
    data >> result_;
}
