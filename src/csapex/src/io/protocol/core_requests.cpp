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
CoreRequests::CoreRequest::CoreRequest(CoreRequestType request_type)
    : RequestImplementation(0),
      request_type_(request_type)
{

}

CoreRequests::CoreRequest::CoreRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}

ResponsePtr CoreRequests::CoreRequest::execute(CsApexCore &core) const
{
    ResponsePtr response;

    switch(request_type_)
    {
    case CoreRequestType::CoreLoad:
        core.load(core.getSettings().get<std::string>("config"));
        break;
    case CoreRequestType::CoreSave:
        core.saveAs(core.getSettings().get<std::string>("config"));
        break;
    case CoreRequestType::SettingsLoad:
        core.getSettings().load();
        break;
    case CoreRequestType::SettingsSave:
        core.getSettings().save();
        break;
    default:
        return std::make_shared<Feedback>(std::string("unknown core request type ") + std::to_string((int)request_type_),
                                          getRequestID());
    }

    return std::make_shared<CoreResponse>(request_type_, getRequestID());
}

void CoreRequests::CoreRequest::serialize(SerializationBuffer &data) const
{
    data << request_type_;
}

void CoreRequests::CoreRequest::deserialize(SerializationBuffer& data)
{
    data >> request_type_;
}

///
/// RESPONSE
///

CoreRequests::CoreResponse::CoreResponse(CoreRequestType request_type, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type)
{

}
CoreRequests::CoreResponse::CoreResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void CoreRequests::CoreResponse::serialize(SerializationBuffer &data) const
{
    data << request_type_;
}

void CoreRequests::CoreResponse::deserialize(SerializationBuffer& data)
{
    data >> request_type_;
}
