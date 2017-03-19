/// HEADER
#include <csapex/io/protcol/request_parameter.h>

/// PROJECT
#include <csapex/serialization/request_serializer.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/serialization/parameter_serializer.h>

/// SYSTEM
#include <iostream>


using namespace csapex;

///
/// REQUEST
///
RequestParameter::ParameterRequest::ParameterRequest(const AUUID &id)
    : id_(id)
{

}

RequestParameter::ParameterRequest::ParameterRequest()
{

}

ResponsePtr RequestParameter::ParameterRequest::execute(CsApexCore &core) const
{
    std::shared_ptr<ParameterResponse> response;

    std::string name = id_.getFullName();
    bool is_setting = name.at(0) == ':';
    if(is_setting) {
        response = std::make_shared<ParameterResponse>(core.getSettings().get(name.substr(1)));
    } else {
        // TODO: get the parameter from the node
    }

    return response;
}

void RequestParameter::ParameterRequest::serialize(SerializationBuffer &data) const
{
    data << id_.getFullName();
}

void RequestParameter::ParameterRequest::deserialize(SerializationBuffer& data)
{
    std::string full_name;
    data >> full_name;
    id_ = AUUID(UUIDProvider::makeUUID_without_parent(full_name));
}

///
/// RESPONSE
///

RequestParameter::ParameterResponse::ParameterResponse(const param::ParameterConstPtr &parameter)
    : param_(parameter)
{

}

RequestParameter::ParameterResponse::ParameterResponse()
{

}

void RequestParameter::ParameterResponse::serialize(SerializationBuffer &data) const
{
    ParameterSerializer::instance().serialize(param_, data);
}

void RequestParameter::ParameterResponse::deserialize(SerializationBuffer& data)
{
    param_ = std::dynamic_pointer_cast<param::Parameter>(ParameterSerializer::instance().deserialize(data));
}

param::ParameterConstPtr RequestParameter::ParameterResponse::getParameter() const
{
    return param_;
}




namespace csapex
{
namespace io
{

class RequestParameterSerializer : public RequestSerializerInterface
{
    virtual void serializeRequest(const RequestConstPtr& packet, SerializationBuffer &data) override
    {
        packet->serialize(data);
    }
    virtual RequestPtr deserializeRequest(SerializationBuffer& data) override
    {
        auto result = std::make_shared<RequestParameter::ParameterRequest>();
        result->deserialize(data);
        return result;
    }
    virtual void serializeResponse(const ResponseConstPtr& packet, SerializationBuffer &data) override
    {
        packet->serialize(data);
    }
    virtual ResponsePtr deserializeResponse(SerializationBuffer& data) override
    {
        auto result = std::make_shared<RequestParameter::ParameterResponse>();
        result->deserialize(data);
        return result;
    }
};
}
RequestSerializerRegistered<io::RequestParameterSerializer> g_register_request_parameter_("RequestParameter");
}

