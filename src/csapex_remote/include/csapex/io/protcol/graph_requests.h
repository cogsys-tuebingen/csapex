#ifndef GRAPH_REQUESTS_H
#define GRAPH_REQUESTS_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

namespace csapex
{
class GraphRequests
{
public:
    enum class GraphRequestType
    {
        GetAllNodes,
        GetAllConnections,

#define HANDLE_ACCESSOR(_enum, type, function) _enum,
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_SIGNAL(_enum, signal)
#include <csapex/model/graph_proxy_accessors.hpp>
    };

    class GraphRequest : public RequestImplementation<GraphRequest>
    {
    public:
        GraphRequest(uint8_t request_id);
        GraphRequest(GraphRequestType request_type, const AUUID& uuid);

        template <typename... Args>
        GraphRequest(GraphRequestType request_type, const AUUID& uuid, Args&&... args) : GraphRequest(request_type, uuid)
        {
            arguments_ = { args... };
        }

        virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        virtual ResponsePtr execute(const SessionPtr& session, CsApexCore& core) const override;

        std::string getType() const override
        {
            return "GraphRequests";
        }

        template <typename R>
        R getArgument(const std::size_t i) const
        {
            return boost::any_cast<R>(arguments_.at(i));
        }

    private:
        GraphRequestType request_type_;
        AUUID uuid_;
        std::vector<boost::any> arguments_;
    };

    class GraphResponse : public ResponseImplementation<GraphResponse>
    {
    public:
        GraphResponse(uint8_t request_id);
        GraphResponse(GraphRequestType request_type, const AUUID& uuid, uint8_t request_id);
        GraphResponse(GraphRequestType request_type, const AUUID& uuid, boost::any result, uint8_t request_id);

        virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        std::string getType() const override
        {
            return "GraphRequests";
        }

        template <typename R>
        R getResult() const
        {
            return boost::any_cast<R>(result_);
        }

    private:
        GraphRequestType request_type_;
        AUUID uuid_;

        boost::any result_;
    };

public:
    using RequestT = GraphRequest;
    using ResponseT = GraphResponse;
};

}  // namespace csapex
#endif  // GRAPH_REQUESTS_H
