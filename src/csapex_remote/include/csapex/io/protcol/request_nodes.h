#ifndef REQUEST_NODES_H
#define REQUEST_NODES_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/model/node_constructor.h>

namespace csapex
{
class RequestNodes
{
public:
    class NodeRequest : public RequestImplementation<NodeRequest>
    {
    public:
        NodeRequest();
        NodeRequest(uint8_t request_id);

        virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        virtual ResponsePtr execute(const SessionPtr& session, CsApexCore& core) const override;

        std::string getType() const override
        {
            return "RequestNodes";
        }
    };

    class NodeResponse : public ResponseImplementation<NodeResponse>
    {
    public:
        NodeResponse(const std::map<std::string, std::vector<NodeConstructorPtr>>& tag_map, uint8_t request_id);
        NodeResponse(uint8_t request_id);

        virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        std::map<std::string, std::vector<NodeConstructorPtr>> getTagMap() const;

        std::string getType() const override
        {
            return "RequestNodes";
        }

    private:
        std::map<std::string, std::vector<NodeConstructorPtr>> tag_map_;
    };

public:
    using RequestT = NodeRequest;
    using ResponseT = NodeResponse;
};

}  // namespace csapex

#endif  // REQUEST_NODES_H
