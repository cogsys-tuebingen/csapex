/// HEADER
#include <csapex/io/protcol/request_nodes.h>

/// PROJECT
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(RequestNodes)

using namespace csapex;

///
/// REQUEST
///
RequestNodes::NodeRequest::NodeRequest()
    : RequestImplementation(0)
{

}

RequestNodes::NodeRequest::NodeRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}

ResponsePtr RequestNodes::NodeRequest::execute(const SessionPtr &session, CsApexCore &core) const
{
    (void) session;
    std::shared_ptr<NodeResponse> response = std::make_shared<NodeResponse>(
                core.getNodeFactory()->getTagMap(),
                getRequestID());
    return response;
}

void RequestNodes::NodeRequest::serialize(SerializationBuffer &data) const
{
    (void) data;
}

void RequestNodes::NodeRequest::deserialize(const SerializationBuffer& data)
{
    (void) data;
}

///
/// RESPONSE
///

RequestNodes::NodeResponse::NodeResponse(const std::map<std::string, std::vector<NodeConstructorPtr> > &tag_map,
                                         uint8_t request_id)
    : ResponseImplementation(request_id),
      tag_map_(tag_map)
{

}
RequestNodes::NodeResponse::NodeResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void RequestNodes::NodeResponse::serialize(SerializationBuffer &data) const
{
    data << tag_map_;
}

void RequestNodes::NodeResponse::deserialize(const SerializationBuffer& data)
{
    data >> tag_map_;
}


std::map<std::string, std::vector<NodeConstructorPtr>> RequestNodes::NodeResponse::getTagMap() const
{
    return tag_map_;
}
