/// HEADER
#include <csapex/io/protcol/profiler_requests.h>

/// PROJECT
#include <csapex/command/command.h>
#include <csapex/io/feedback.h>
#include <csapex/io/raw_message.h>
#include <csapex/io/session.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node_characteristics.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node_handle.h>
#include <csapex/profiling/profiler.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(ProfilerRequests)

using namespace csapex;

///
/// REQUEST
///
ProfilerRequests::ProfilerRequest::ProfilerRequest(ProfilerRequestType request_type, const AUUID &uuid)
    : RequestImplementation(0),
      request_type_(request_type),
      uuid_(uuid)
{

}

ProfilerRequests::ProfilerRequest::ProfilerRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}

ResponsePtr ProfilerRequests::ProfilerRequest::execute(const SessionPtr &session, CsApexCore &core) const
{
    NodeFacadePtr nf = core.getRoot()->getLocalGraph()->findNodeFacade(uuid_);
    NodeFacadeImplementationPtr nf_local = std::dynamic_pointer_cast<NodeFacadeImplementation>(nf);
    apex_assert_hard(nf_local);
    NodeHandlePtr nh = nf_local->getNodeHandle();

    ProfilerPtr pf = nf->getProfiler();

    switch(request_type_)
    {
    case ProfilerRequestType::SetEnabled:
        pf->setEnabled(getArgument<bool>(0));
        break;

    default:
        return std::make_shared<Feedback>(std::string("unknown profiler request type ") + std::to_string((int)request_type_),
                                          getRequestID());
    }

    return std::make_shared<ProfilerResponse>(request_type_, uuid_, getRequestID());
}

void ProfilerRequests::ProfilerRequest::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << uuid_;
    data << arguments_;
}

void ProfilerRequests::ProfilerRequest::deserialize(const SerializationBuffer& data)
{
    data >> request_type_;
    data >> uuid_;
    data >> arguments_;
}

///
/// RESPONSE
///

ProfilerRequests::ProfilerResponse::ProfilerResponse(ProfilerRequestType request_type, const AUUID& uuid, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      uuid_(uuid)
{

}
ProfilerRequests::ProfilerResponse::ProfilerResponse(ProfilerRequestType request_type, const AUUID& uuid, boost::any result, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      uuid_(uuid),
      result_(result)
{

}

ProfilerRequests::ProfilerResponse::ProfilerResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void ProfilerRequests::ProfilerResponse::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << uuid_;
    data << result_;
}

void ProfilerRequests::ProfilerResponse::deserialize(const SerializationBuffer& data)
{
    data >> request_type_;
    data >> uuid_;
    data >> result_;
}
