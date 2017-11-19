/// HEADER
#include <csapex/io/protcol/node_requests.h>

/// PROJECT
#include <csapex/command/command.h>
#include <csapex/io/feedback.h>
#include <csapex/io/raw_message.h>
#include <csapex/io/session.h>
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/node_characteristics.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(NodeRequests)

using namespace csapex;

///
/// REQUEST
///
NodeRequests::NodeRequest::NodeRequest(NodeRequestType request_type, const AUUID &uuid)
    : RequestImplementation(0),
      request_type_(request_type),
      uuid_(uuid)
{

}

NodeRequests::NodeRequest::NodeRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}

ResponsePtr NodeRequests::NodeRequest::execute(const SessionPtr &session, CsApexCore &core) const
{
    NodeFacadePtr nf = core.getRoot()->getLocalGraph()->findNodeFacade(uuid_);
    NodeFacadeLocalPtr nf_local = std::dynamic_pointer_cast<NodeFacadeLocal>(nf);
    apex_assert_hard(nf_local);
    NodeHandlePtr nh = nf_local->getNodeHandle();

    switch(request_type_)
    {
    case NodeRequestType::AddClient:
    {
        nh->remote_data_connection.connect([session](StreamableConstPtr data){
            if(RawMessageConstPtr msg = std::dynamic_pointer_cast<RawMessage const>(data)) {
                session->write(msg);
            } else {
                // TODO: what to do here? we need a uuid
            }
        });
        NodeHandle* nhp = nh.get();
        session->stopped.connect([nhp](){
            // TODO: disconnect only the appropriate client...
            nhp->remote_data_connection.disconnectAll();
        });
    }
        break;
    case NodeRequestType::RemoveClient:
    {
        // TODO: disconnect only the appropriate client...
        nh->remote_data_connection.disconnectAll();
    }
        break;

    case NodeRequestType::GetParameters:
    {
        auto result = nf->getParameters();
        return std::make_shared<NodeResponse>(request_type_, uuid_, result, getRequestID());
    }
    case NodeRequestType::GetNodeState:
    {
        auto result = nf->getNodeState();
        return std::make_shared<NodeResponse>(request_type_, uuid_, result, getRequestID());
    }

    case NodeRequestType::SetProfiling:
        nf->setProfiling(getArgument<bool>(0));
        break;

    case NodeRequestType::GetLoggerOutput:
    {
        auto result = nf->getLoggerOutput(getArgument<ErrorState::ErrorLevel>(0));
        return std::make_shared<NodeResponse>(request_type_, uuid_, result, getRequestID());
    }



    case NodeRequestType::IsParameterInput:
    {
        auto result = nf->isParameterInput(getArgument<UUID>(0));
        return std::make_shared<NodeResponse>(request_type_, uuid_, result, getRequestID());
    }

    case NodeRequestType::IsParameterOutput:
    {
        auto result = nf->isParameterOutput(getArgument<UUID>(0));
        return std::make_shared<NodeResponse>(request_type_, uuid_, result, getRequestID());
    }

        /**
         * begin: generate cases
         **/
#define HANDLE_ACCESSOR(_enum, type, function) \
    case NodeRequests::NodeRequestType::_enum:\
        return std::make_shared<NodeResponse>(request_type_, uuid_, nf->function(), getRequestID());
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_SIGNAL(_enum, signal)

    #include <csapex/model/node_facade_remote_accessors.hpp>
        /**
         * end: generate cases
         **/


    default:
        return std::make_shared<Feedback>(std::string("unknown node request type ") + std::to_string((int)request_type_),
                                          getRequestID());
    }

    return std::make_shared<NodeResponse>(request_type_, uuid_, getRequestID());
}

void NodeRequests::NodeRequest::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << uuid_;
    data << arguments_;
}

void NodeRequests::NodeRequest::deserialize(const SerializationBuffer& data)
{
    data >> request_type_;
    data >> uuid_;
    data >> arguments_;
}

///
/// RESPONSE
///

NodeRequests::NodeResponse::NodeResponse(NodeRequestType request_type, const AUUID& uuid, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      uuid_(uuid)
{

}
NodeRequests::NodeResponse::NodeResponse(NodeRequestType request_type, const AUUID& uuid, boost::any result, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      uuid_(uuid),
      result_(result)
{

}

NodeRequests::NodeResponse::NodeResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void NodeRequests::NodeResponse::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << uuid_;
    data << result_;
}

void NodeRequests::NodeResponse::deserialize(const SerializationBuffer& data)
{
    data >> request_type_;
    data >> uuid_;
    data >> result_;
}
