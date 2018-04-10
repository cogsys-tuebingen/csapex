/// HEADER
#include <csapex/io/protcol/core_requests.h>

/// PROJECT
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/io/feedback.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/snippet.h>

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

ResponsePtr CoreRequests::CoreRequest::execute(const SessionPtr &session, CsApexCore &core) const
{
    switch(request_type_)
    {
    case CoreRequestType::SettingsSavePersistent:
        core.getSettings().savePersistent();
        break;
    case CoreRequestType::SettingsLoadPersistent:
        core.getSettings().loadPersistent();
        break;
    case CoreRequestType::CoreSave: {
        int args = parameters_.size();
        if(args == 0) {
            core.saveAs(core.getSettings().get<std::string>("config"));

        } else if(args == 1) {
            core.saveAs(boost::any_cast<std::string>(parameters_.at(0)));
        } else {
            core.saveAs(boost::any_cast<std::string>(parameters_.at(0)), boost::any_cast<bool>(parameters_.at(1)));
        }
    }
        break;
    case CoreRequestType::CoreLoad:{
        int args = parameters_.size();
        if(args == 0) {
            core.load(core.getSettings().get<std::string>("config"));

        } else {
            core.load(boost::any_cast<std::string>(parameters_.at(0)));
        }
    }
        break;
    case CoreRequestType::CoreSerialize: {
        int args = parameters_.size();
        if(args == 2) {
            SnippetPtr snippet = core.serializeNodes(boost::any_cast<AUUID>(parameters_.at(0)),
                                                     boost::any_cast<std::vector<UUID>>(parameters_.at(1)));
            return std::make_shared<CoreResponse>(request_type_, snippet, getRequestID());
        }
    }
        break;
    case CoreRequestType::CoreStep:
        core.step();
        break;
    case CoreRequestType::CoreShutdown:
        core.shutdown();
        break;
    case CoreRequestType::CoreResetActivity:
        core.getRoot()->resetActivity();
        break;
    case CoreRequestType::CoreClearBlock:
        core.getRoot()->clearBlock();
        break;
    case CoreRequestType::CoreReset:
        core.reset();
        break;
    case CoreRequestType::CoreSetSteppingMode:
        core.setSteppingMode(boost::any_cast<bool>(parameters_.at(0)));
        break;
    case CoreRequestType::CoreSetPause:
        core.setPause(boost::any_cast<bool>(parameters_.at(0)));
        break;
    case CoreRequestType::CoreSendNotification:
        core.sendNotification(boost::any_cast<std::string>(parameters_.at(0)), static_cast<ErrorState::ErrorLevel>(boost::any_cast<uint8_t>(parameters_.at(1))));
        break;

    case CoreRequestType::CoreGetSteppingMode:
        return std::make_shared<CoreResponse>(request_type_, core.isSteppingMode(), getRequestID());
    case CoreRequestType::CoreGetPause:
        return std::make_shared<CoreResponse>(request_type_, core.isPaused(), getRequestID());

    default:
        return std::make_shared<Feedback>(std::string("unknown core request type ") + std::to_string((int)request_type_),
                                          getRequestID());
    }

    return std::make_shared<CoreResponse>(request_type_, getRequestID());
}

void CoreRequests::CoreRequest::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << parameters_;
}

void CoreRequests::CoreRequest::deserialize(const SerializationBuffer& data)
{
    data >> request_type_;
    data >> parameters_;
}

///
/// RESPONSE
///

CoreRequests::CoreResponse::CoreResponse(CoreRequestType request_type, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type)
{

}
CoreRequests::CoreResponse::CoreResponse(CoreRequestType request_type, boost::any result, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      result_(result)
{

}
CoreRequests::CoreResponse::CoreResponse(uint8_t request_id)
    : ResponseImplementation(request_id)
{

}

void CoreRequests::CoreResponse::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << result_;
}

void CoreRequests::CoreResponse::deserialize(const SerializationBuffer& data)
{
    data >> request_type_;
    data >> result_;
}
