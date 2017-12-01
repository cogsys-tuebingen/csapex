/// HEADER
#include <csapex/io/protcol/connector_notes.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/serialization/note_serializer.h>

CSAPEX_REGISTER_NOTE_SERIALIZER(ConnectorNote)

using namespace csapex;

ConnectorNote::ConnectorNote()
{
}

ConnectorNote::ConnectorNote(ConnectorNoteType request_type, const AUUID &uuid)
    : NoteImplementation(uuid),
      note_type_(request_type)
{

}

ConnectorNote::ConnectorNote(ConnectorNoteType request_type, const AUUID &uuid, const boost::any &payload)
    : NoteImplementation(uuid),
      note_type_(request_type),
      payload_(payload)
{

}

void ConnectorNote::serialize(SerializationBuffer &data) const
{
    Note::serialize(data);

    data << note_type_;
    data << payload_;
}

void ConnectorNote::deserialize(const SerializationBuffer& data)
{
    Note::deserialize(data);

    data >> note_type_;
    data >> payload_;
}
