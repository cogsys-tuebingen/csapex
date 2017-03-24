/// HEADER
#include <csapex/io/protcol/core_requests.h>

/// PROJECT
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/io/feedback.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/serialization/parameter_serializer.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(CoreRequests)

using namespace csapex;

///
/// REQUEST
///
CoreRequests::CoreRequest::CoreRequest(CoreRequestTarget request_target, CoreRequestType request_type)
    : RequestImplementation(0),
      request_target_(request_target), request_type_(request_type)
{

}

CoreRequests::CoreRequest::CoreRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}

ResponsePtr CoreRequests::CoreRequest::execute(CsApexCore &core) const
{
    ResponsePtr response;

    switch(request_target_)
    {
    case CoreRequestTarget::Core:
        executeCore(core);

        break;
    case CoreRequestTarget::Settings:
        executeSettings(core);

        break;
    default:
        response = std::make_shared<Feedback>(std::string("unknown request target ") + std::to_string((int)request_target_),
                                              getRequestID());
        break;
    }

    if(!response) {
        response = std::make_shared<CoreResponse>(request_target_, request_type_, getRequestID());
    }
    return response;
}

ResponsePtr CoreRequests::CoreRequest::executeCore(CsApexCore &core) const
{
    ResponsePtr response;

    switch(request_type_)
    {
    case CoreRequestType::Load:
        core.load(core.getSettings().get<std::string>("config"));
        break;
    case CoreRequestType::Save:
        core.saveAs(core.getSettings().get<std::string>("config"));
        break;
    default:
        response = std::make_shared<Feedback>(std::string("unknown settings request type ") + std::to_string((int)request_type_),
                                              getRequestID());
        break;
    }

    return response;
}

ResponsePtr CoreRequests::CoreRequest::executeSettings(CsApexCore &core) const
{
    ResponsePtr response;

    switch(request_type_)
    {
    case CoreRequestType::Load:
        core.getSettings().load();
        break;
    case CoreRequestType::Save:
        core.getSettings().save();
        break;
    default:
        response = std::make_shared<Feedback>(std::string("unknown settings request type ") + std::to_string((int)request_type_),
                                              getRequestID());
        break;
    }

    return response;
}

void CoreRequests::CoreRequest::serialize(SerializationBuffer &data) const
{
    data << request_target_;
    data << request_type_;
}

void CoreRequests::CoreRequest::deserialize(SerializationBuffer& data)
{
    data >> request_target_;
    data >> request_type_;
}

///
/// RESPONSE
///

CoreRequests::CoreResponse::CoreResponse(CoreRequestTarget request_target, CoreRequestType request_type, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_target_(request_target), request_type_(request_type)
{

}
CoreRequests::CoreResponse::CoreResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void CoreRequests::CoreResponse::serialize(SerializationBuffer &data) const
{
    data << request_target_;
    data << request_type_;
}

void CoreRequests::CoreResponse::deserialize(SerializationBuffer& data)
{
    data >> request_target_;
    data >> request_type_;
}
