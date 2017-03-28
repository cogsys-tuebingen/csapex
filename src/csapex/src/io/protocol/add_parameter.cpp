/// HEADER
#include <csapex/io/protcol/add_parameter.h>

/// PROJECT
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/serialization/parameter_serializer.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(AddParameter)

using namespace csapex;

///
/// REQUEST
///
AddParameter::ParameterRequest::ParameterRequest(const AUUID &id, const std::string& name, const std::string& description,
                                                 boost::any value, bool persistent)
    : RequestImplementation(0),
      id_(id), name_(name), description_(description),
      value_(value), persistent_(persistent)
{
    apex_assert_hard(!name_.empty());
}

AddParameter::ParameterRequest::ParameterRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}

ResponsePtr AddParameter::ParameterRequest::execute(CsApexCore &core) const
{
    std::shared_ptr<ParameterResponse> response;

    apex_assert_hard(!name_.empty());
    param::Parameter::Ptr param = std::make_shared<param::ValueParameter>(name_, param::ParameterDescription(description_));
    param->set_unsafe(value_);

    if(id_.global()) {
        core.getSettings().add(param, persistent_);
        response = std::make_shared<ParameterResponse>(param, getRequestID());
    } else {
        // TODO: get the parameter from the node
    }

    return response;
}

void AddParameter::ParameterRequest::serialize(SerializationBuffer &data) const
{
    data << id_;
    data << name_;
    data << description_;
    data << value_;
    data << persistent_;
}

void AddParameter::ParameterRequest::deserialize(SerializationBuffer& data)
{
    data >> id_;
    data >> name_;
    data >> description_;
    data >> value_;
    data >> persistent_;
}

///
/// RESPONSE
///

AddParameter::ParameterResponse::ParameterResponse(const param::ParameterConstPtr &parameter, uint8_t request_id)
    : ResponseImplementation(request_id), param_(parameter)
{

}
AddParameter::ParameterResponse::ParameterResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void AddParameter::ParameterResponse::serialize(SerializationBuffer &data) const
{
    ParameterSerializer::instance().serialize(param_, data);
}

void AddParameter::ParameterResponse::deserialize(SerializationBuffer& data)
{
    param_ = std::dynamic_pointer_cast<param::Parameter>(ParameterSerializer::instance().deserialize(data));
}

param::ParameterConstPtr AddParameter::ParameterResponse::getParameter() const
{
    return param_;
}
