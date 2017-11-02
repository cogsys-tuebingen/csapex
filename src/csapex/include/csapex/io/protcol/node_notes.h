#ifndef NODE_NOTES_H
#define NODE_NOTES_H

/// PROJECT
#include <csapex/io/note_impl.hpp>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{

enum class NodeNoteType
{
    /**
     * begin: connect signals
     **/
    #define HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
        function##Changed,
    #define HANDLE_SIGNAL(_enum, signal) \
        _enum##Triggered,
    #include <csapex/model/node_facade_remote_accessors.hpp>
    /**
     * end: connect signals
     **/

    ParameterAddedTriggered,
    ParameterChangedTriggered,
    ParameterRemovedTriggered,

    NodeStateChanged,

    ProfilingStartTriggered,
    ProfilingStopTriggered,

    IntervalStartTriggered,
    IntervalEndTriggered,

    ConnectorCreatedTriggered,
    ConnectorRemovedTriggered,

    ConnectionStartTriggered,
    ConnectionCreatedTriggered,
    ConnectionRemovedTriggered,

    ErrorEvent,
    Notification
};


class NodeNote : public NoteImplementation<NodeNote>
{
public:
    NodeNote();
    NodeNote(NodeNoteType request_type, const AUUID& uuid);
    NodeNote(NodeNoteType request_type, const AUUID& uuid, const std::vector<boost::any>& payload);

    template <typename... Args>
    NodeNote(NodeNoteType request_type, const AUUID& uuid, Args... args)
        : NodeNote(request_type, uuid, {std::forward<Args>(args)...})
    {
    }

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(const SerializationBuffer& data) override;

    NodeNoteType getNoteType() const
    {
        return note_type_;
    }

    std::size_t countPayload() const
    {
        return payload_.size();
    }

    template <typename T>
    T getPayload(const std::size_t pos) const
    {
        return boost::any_cast<T>(payload_.at(pos));
    }

private:
    NodeNoteType note_type_;
    std::vector<boost::any> payload_;
};

}

#endif // NODE_NOTES_H
