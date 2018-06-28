#ifndef COMMAND_BROADCASTS_H
#define COMMAND_BROADCASTS_H

/// PROJECT
#include <csapex/io/broadcast_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

namespace csapex
{

class CommandBroadcasts : public BroadcastImplementation<CommandBroadcasts>
{
public:

    enum class CommandBroadcastType
    {
        None,

        StateChanged,
        DirtyChanged
    };

    CommandBroadcasts();
    CommandBroadcasts(CommandBroadcastType Broadcast_type);
    CommandBroadcasts(CommandBroadcastType Broadcast_type, bool flag_);

    virtual void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    std::string getType() const override
    {
        return "CommandBroadcasts";
    }

    CommandBroadcastType getBroadcastType() const;
    bool getFlag() const;

private:
    CommandBroadcastType broadcast_type_;

    bool flag_;

};

}

#endif // COMMAND_BROADCASTS_H
