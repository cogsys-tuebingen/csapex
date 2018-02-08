/// HEADER
#include <csapex/io/protcol/request_parameter.h>

/// PROJECT
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/serialization/parameter_serializer.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(RequestParameter)

using namespace csapex;

///
/// REQUEST
///
RequestParameter::ParameterRequest::ParameterRequest(const AUUID &id)
    : RequestImplementation(0), id_(id)
{

}

RequestParameter::ParameterRequest::ParameterRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}

ResponsePtr RequestParameter::ParameterRequest::execute(const SessionPtr &session, CsApexCore &core) const
{
    std::shared_ptr<ParameterResponse> response;

    if(id_.global()) {
        apex_assert_hard(!id_.globalName().empty());
        auto param = core.getSettings().getNoThrow(id_.globalName());
        response = std::make_shared<ParameterResponse>(param, getRequestID());
    } else {
        // TODO: get the parameter from the node
    }

    return response;
}

void RequestParameter::ParameterRequest::serialize(SerializationBuffer &data) const
{
    data << id_;
}

void RequestParameter::ParameterRequest::deserialize(const SerializationBuffer& data)
{
    data >> id_;
}

///
/// RESPONSE
///

RequestParameter::ParameterResponse::ParameterResponse(const param::ParameterConstPtr &parameter, uint8_t request_id)
    : ResponseImplementation(request_id), param_(parameter)
{

}
RequestParameter::ParameterResponse::ParameterResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void RequestParameter::ParameterResponse::serialize(SerializationBuffer &data) const
{
    ParameterSerializer::instance().serialize(*param_, data);
}

void RequestParameter::ParameterResponse::deserialize(const SerializationBuffer& data)
{
    param_ = std::dynamic_pointer_cast<param::Parameter>(ParameterSerializer::instance().deserialize(data));
}

param::ParameterConstPtr RequestParameter::ParameterResponse::getParameter() const
{
    return param_;
}
