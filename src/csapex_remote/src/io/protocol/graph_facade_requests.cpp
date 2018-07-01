/// HEADER
#include <csapex/io/protcol/graph_facade_requests.h>

/// PROJECT
#include <csapex/command/command.h>
#include <csapex/io/feedback.h>
#include <csapex/io/raw_message.h>
#include <csapex/io/session.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(GraphFacadeRequests)

using namespace csapex;

///
/// REQUEST
///
GraphFacadeRequests::GraphFacadeRequest::GraphFacadeRequest(GraphFacadeRequestType request_type, const AUUID& uuid) : RequestImplementation(0), request_type_(request_type), uuid_(uuid)
{
}

GraphFacadeRequests::GraphFacadeRequest::GraphFacadeRequest(uint8_t request_id) : RequestImplementation(request_id)
{
}

ResponsePtr GraphFacadeRequests::GraphFacadeRequest::execute(const SessionPtr& session, CsApexCore& core) const
{
    GraphFacadePtr gf = uuid_.empty() ? core.getRoot() : core.getRoot()->getSubGraph(uuid_);
    GraphFacadeImplementationPtr gf_local = std::dynamic_pointer_cast<GraphFacadeImplementation>(gf);
    apex_assert_hard(gf_local);

    switch (request_type_) {
        case GraphFacadeRequestType::SetPause: {
            gf->pauseRequest(getArgument<bool>(0));
        } break;
        case GraphFacadeRequestType::ResetActivity: {
            gf->resetActivity();
        } break;
        case GraphFacadeRequestType::ClearBlock: {
            gf->clearBlock();
        } break;
        case GraphFacadeRequestType::GenerateUUID: {
            UUID result = gf->generateUUID(getArgument<std::string>(0));
            return std::make_shared<GraphFacadeResponse>(request_type_, uuid_, result, getRequestID());
        } break;

        /**
         * begin: generate cases
         **/
#define HANDLE_ACCESSOR(_enum, type, function)                                                                                                                                                         \
    case GraphFacadeRequests::GraphFacadeRequestType::_enum:                                                                                                                                           \
        return std::make_shared<GraphFacadeResponse>(request_type_, uuid_, gf->function(), getRequestID());
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_facade_proxy_accessors.hpp>
            /**
             * end: generate cases
             **/

        default:
            return std::make_shared<Feedback>(std::string("unknown graph facade request type ") + std::to_string((int)request_type_), getRequestID());
    }

    return std::make_shared<GraphFacadeResponse>(request_type_, uuid_, getRequestID());
}

void GraphFacadeRequests::GraphFacadeRequest::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    data << request_type_;
    data << uuid_;
    data << arguments_;
}

void GraphFacadeRequests::GraphFacadeRequest::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> request_type_;
    data >> uuid_;
    data >> arguments_;
}

///
/// RESPONSE
///

GraphFacadeRequests::GraphFacadeResponse::GraphFacadeResponse(GraphFacadeRequestType request_type, const AUUID& uuid, uint8_t request_id)
  : ResponseImplementation(request_id), request_type_(request_type), uuid_(uuid)
{
}
GraphFacadeRequests::GraphFacadeResponse::GraphFacadeResponse(GraphFacadeRequestType request_type, const AUUID& uuid, boost::any result, uint8_t request_id)
  : ResponseImplementation(request_id), request_type_(request_type), uuid_(uuid), result_(result)
{
}

GraphFacadeRequests::GraphFacadeResponse::GraphFacadeResponse(uint8_t request_id) : ResponseImplementation(request_id)
{
}

void GraphFacadeRequests::GraphFacadeResponse::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    data << request_type_;
    data << uuid_;
    data << result_;
}

void GraphFacadeRequests::GraphFacadeResponse::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> request_type_;
    data >> uuid_;
    data >> result_;
}
