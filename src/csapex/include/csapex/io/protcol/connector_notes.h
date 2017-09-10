#ifndef CONNECTOR_NOTES_H
#define CONNECTOR_NOTES_H

/// PROJECT
#include <csapex/io/note_impl.hpp>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{

enum class ConnectorNoteType
{
    /**
     * begin: connect signals
     **/
    #define HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
        function##Changed,
    #include <csapex/model/connector_remote_accessors.hpp>
    /**
     * end: connect signals
     **/
};


class ConnectorNote : public NoteImplementation<ConnectorNote>
{
public:
    ConnectorNote();
    ConnectorNote(ConnectorNoteType request_type, const AUUID& uuid);
    ConnectorNote(ConnectorNoteType request_type, const AUUID& uuid, const boost::any& payload);

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(SerializationBuffer& data) override;

    ConnectorNoteType getNoteType() const
    {
        return note_type_;
    }

    boost::any getPayload() const
    {
        return payload_;
    }

private:
    ConnectorNoteType note_type_;
    boost::any payload_;
};

}

#endif // CONNECTOR_NOTES_H
