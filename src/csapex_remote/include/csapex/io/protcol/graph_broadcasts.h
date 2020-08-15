#ifndef GRAPH_BROADCASTS_H
#define GRAPH_BROADCASTS_H

/// PROJECT
#include <csapex/io/broadcast_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/any.h>

namespace csapex
{
class GraphBroadcasts : public BroadcastImplementation<GraphBroadcasts>
{
public:
    enum class GraphBroadcastType
    {
        None
    };

    GraphBroadcasts();
    GraphBroadcasts(GraphBroadcastType Broadcast_type);
    GraphBroadcasts(GraphBroadcastType Broadcast_type, AUUID uuid);
    GraphBroadcasts(GraphBroadcastType Broadcast_type, AUUID uuid, std::any payload);

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    std::string getType() const override
    {
        return "GraphBroadcasts";
    }

    GraphBroadcastType getBroadcastType() const;
    AUUID getGraphUUID() const;

    template <typename R>
    R getPayload() const
    {
        return std::any_cast<R>(payload_);
    }

private:
    GraphBroadcastType broadcast_type_;

    AUUID graph_uuid_;

    std::any payload_;
};

}  // namespace csapex

#endif  // GRAPH_BROADCASTS_H
