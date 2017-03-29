#ifndef COMMAND_REQUESTS_H
#define COMMAND_REQUESTS_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

namespace csapex
{

class CommandRequests
{
public:

    enum class CommandRequestType
    {
        Execute,
        ExecuteLater,

        IsDirty,
        CanUndo,
        CanRedo,

        Undo,
        Redo
    };

    class CommandRequest : public RequestImplementation<CommandRequest>
    {
    public:
        CommandRequest(uint8_t request_id);
        CommandRequest(CommandRequestType request_type);

        CommandRequest(CommandRequestType request_type, const CommandPtr& param)
            : CommandRequest(request_type)
        {
            command_ = param;
        }

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        virtual ResponsePtr execute(CsApexCore& core) const override;

        std::string getType() const override
        {
            return "CommandRequests";
        }

    private:
        CommandRequestType request_type_;

        CommandPtr command_;
    };


    class CommandResponse : public ResponseImplementation<CommandResponse>
    {
    public:
        CommandResponse(uint8_t request_id);
        CommandResponse(CommandRequestType request_type, uint8_t request_id);
        CommandResponse(CommandRequestType request_type, bool result, uint8_t request_id);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        bool getResult() const
        {
            return result_;
        }

        std::string getType() const override
        {
            return "CommandRequests";
        }

    private:
        CommandRequestType request_type_;

        bool result_;
    };


public:
    using RequestT = CommandRequest;
    using ResponseT = CommandResponse;
};

}

#endif // COMMAND_REQUESTS_H
