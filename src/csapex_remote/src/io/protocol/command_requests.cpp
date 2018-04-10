/// HEADER
#include <csapex/io/protcol/command_requests.h>

/// PROJECT
#include <csapex/serialization/request_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/io/feedback.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/graph_facade.h>
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/command/command.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_REQUEST_SERIALIZER(CommandRequests)

using namespace csapex;

///
/// REQUEST
///
CommandRequests::CommandRequest::CommandRequest(CommandRequestType request_type)
    : RequestImplementation(0),
      request_type_(request_type)
{

}

CommandRequests::CommandRequest::CommandRequest(uint8_t request_id)
    : RequestImplementation(request_id)
{

}

ResponsePtr CommandRequests::CommandRequest::execute(const SessionPtr &session, CsApexCore &core) const
{
    CommandDispatcherPtr dispatcher = core.getCommandDispatcher();
    switch(request_type_)
    {
    case CommandRequestType::Execute:
        dispatcher->execute(command_);
        break;
    case CommandRequestType::ExecuteLater:
        if(command_) {
            dispatcher->executeLater(command_);
        } else {
            dispatcher->executeLater();
        }
        break;

    case CommandRequestType::IsDirty:
        return std::make_shared<CommandResponse>(request_type_, dispatcher->isDirty(), getRequestID());

    case CommandRequestType::CanUndo:
        return std::make_shared<CommandResponse>(request_type_, dispatcher->canUndo(), getRequestID());

    case CommandRequestType::CanRedo:
        return std::make_shared<CommandResponse>(request_type_, dispatcher->canRedo(), getRequestID());


    case CommandRequestType::Undo:
        dispatcher->undo();
        break;

    case CommandRequestType::Redo:
        dispatcher->redo();
        break;


    default:
        return std::make_shared<Feedback>(std::string("unknown core request type ") + std::to_string((int)request_type_),
                                          getRequestID());
    }

    return std::make_shared<CommandResponse>(request_type_, getRequestID());
}

void CommandRequests::CommandRequest::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << command_;
}

void CommandRequests::CommandRequest::deserialize(const SerializationBuffer& data)
{
    data >> request_type_;
    data >> command_;
}

///
/// RESPONSE
///

CommandRequests::CommandResponse::CommandResponse(CommandRequestType request_type, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      result_(false)
{

}
CommandRequests::CommandResponse::CommandResponse(CommandRequestType request_type, bool result, uint8_t request_id)
    : ResponseImplementation(request_id),
      request_type_(request_type),
      result_(result)
{

}
CommandRequests::CommandResponse::CommandResponse(uint8_t request_id)
    : ResponseImplementation(request_id),
      result_(false)
{

}

void CommandRequests::CommandResponse::serialize(SerializationBuffer &data) const
{
    data << request_type_;
    data << result_;
}

void CommandRequests::CommandResponse::deserialize(const SerializationBuffer& data)
{
    data >> request_type_;
    data >> result_;
}
