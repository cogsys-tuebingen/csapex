#ifndef NODE_REQUESTS_H
#define NODE_REQUESTS_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

namespace csapex
{

class NodeRequests
{
public:

    enum class NodeRequestType
    {
        AddClient,
        RemoveClient,

        SetProfiling,
        GetLoggerOutput,


        IsParameterInput,
        IsParameterOutput,

#define HANDLE_ACCESSOR(_enum, type, function) _enum,
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
#include <csapex/model/node_facade_remote_accessors.hpp>
    };

    class NodeRequest : public RequestImplementation<NodeRequest>
    {
    public:
        NodeRequest(uint8_t request_id);
        NodeRequest(NodeRequestType request_type, const AUUID& uuid);

        template <typename... Args>
        NodeRequest(NodeRequestType request_type, const AUUID& uuid, Args&&... args)
            : NodeRequest(request_type, uuid)
        {
            arguments_ = { args... };
        }

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        virtual ResponsePtr execute(const SessionPtr& session, CsApexCore& core) const override;

        std::string getType() const override
        {
            return "NodeRequests";
        }

        template <typename R>
        R getArgument(const std::size_t i) const
        {
            return boost::any_cast<R>(arguments_.at(i));
        }

    private:
        NodeRequestType request_type_;
        AUUID uuid_;
        std::vector<boost::any> arguments_;
    };


    class NodeResponse : public ResponseImplementation<NodeResponse>
    {
    public:
        NodeResponse(uint8_t request_id);
        NodeResponse(NodeRequestType request_type, const AUUID& uuid, uint8_t request_id);
        NodeResponse(NodeRequestType request_type, const AUUID& uuid, boost::any result, uint8_t request_id);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        std::string getType() const override
        {
            return "NodeRequests";
        }        

        template <typename R>
        R getResult() const
        {
            return boost::any_cast<R>(result_);
        }


    private:
        NodeRequestType request_type_;
        AUUID uuid_;

        boost::any result_;
    };


public:
    using RequestT = NodeRequest;
    using ResponseT = NodeResponse;
};

}
#endif // NODE_REQUESTS_H
