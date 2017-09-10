#ifndef GRAPH_BROADCASTS_H
#define GRAPH_BROADCASTS_H

/// PROJECT
#include <csapex/io/broadcast_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{

class GraphBroadcasts : public BroadcastImplementation<GraphBroadcasts>
{
public:

    enum class GraphBroadcastType
    {
        None,

        GraphCreated,
        GraphDestroyed,

        NodeCreated,
        NodeDestroyed
    };

    GraphBroadcasts();
    GraphBroadcasts(GraphBroadcastType Broadcast_type);
    GraphBroadcasts(GraphBroadcastType Broadcast_type, AUUID uuid);
    GraphBroadcasts(GraphBroadcastType Broadcast_type, AUUID uuid, boost::any payload);

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(SerializationBuffer& data) override;

    std::string getType() const override
    {
        return "GraphBroadcasts";
    }

    GraphBroadcastType getBroadcastType() const;
    AUUID getGraphUUID() const;

    template <typename R>
    R getPayload() const
    {
        return boost::any_cast<R>(payload_);
    }


private:
    GraphBroadcastType broadcast_type_;

    AUUID graph_uuid_;

    boost::any payload_;
};

}

#endif // GRAPH_BROADCASTS_H
