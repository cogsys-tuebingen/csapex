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
};


class NodeNote : public NoteImplementation<NodeNote>
{
public:
    NodeNote();
    NodeNote(NodeNoteType request_type, const AUUID& uuid);
    NodeNote(NodeNoteType request_type, const AUUID& uuid, const boost::any& payload);

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(SerializationBuffer& data) override;

    NodeNoteType getNoteType() const
    {
        return note_type_;
    }

    boost::any getPayload() const
    {
        return payload_;
    }

private:
    NodeNoteType note_type_;
    boost::any payload_;
};

}

#endif // NODE_NOTES_H
