#ifndef GRAPH_FACADE_REQUESTS_H
#define GRAPH_FACADE_REQUESTS_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

namespace csapex
{

class GraphFacadeRequests
{
public:

    enum class GraphFacadeRequestType
    {
        SetPause,
        ResetActivity,
        ClearBlock,

        GenerateUUID,

#define HANDLE_ACCESSOR(_enum, type, function) _enum,
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_SIGNAL(_enum, signal)
#include <csapex/model/graph_facade_proxy_accessors.hpp>
    };

    class GraphFacadeRequest : public RequestImplementation<GraphFacadeRequest>
    {
    public:
        GraphFacadeRequest(uint8_t request_id);
        GraphFacadeRequest(GraphFacadeRequestType request_type, const AUUID& uuid);

        template <typename... Args>
        GraphFacadeRequest(GraphFacadeRequestType request_type, const AUUID& uuid, Args&&... args)
            : GraphFacadeRequest(request_type, uuid)
        {
            arguments_ = { args... };
        }

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(const SerializationBuffer& data) override;

        virtual ResponsePtr execute(const SessionPtr& session, CsApexCore& core) const override;

        std::string getType() const override
        {
            return "GraphFacadeRequests";
        }

        template <typename R>
        R getArgument(const std::size_t i) const
        {
            return boost::any_cast<R>(arguments_.at(i));
        }

    private:
        GraphFacadeRequestType request_type_;
        AUUID uuid_;
        std::vector<boost::any> arguments_;
    };


    class GraphFacadeResponse : public ResponseImplementation<GraphFacadeResponse>
    {
    public:
        GraphFacadeResponse(uint8_t request_id);
        GraphFacadeResponse(GraphFacadeRequestType request_type, const AUUID& uuid, uint8_t request_id);
        GraphFacadeResponse(GraphFacadeRequestType request_type, const AUUID& uuid, boost::any result, uint8_t request_id);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(const SerializationBuffer& data) override;

        std::string getType() const override
        {
            return "GraphFacadeRequests";
        }

        template <typename R>
        R getResult() const
        {
            return boost::any_cast<R>(result_);
        }


    private:
        GraphFacadeRequestType request_type_;
        AUUID uuid_;

        boost::any result_;
    };


public:
    using RequestT = GraphFacadeRequest;
    using ResponseT = GraphFacadeResponse;
};

}
#endif // GRAPH_FACADE_REQUESTS_H
