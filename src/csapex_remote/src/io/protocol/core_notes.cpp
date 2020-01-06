/// HEADER
#include <csapex/io/protcol/core_notes.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/note_serializer.h>

CSAPEX_REGISTER_NOTE_SERIALIZER(CoreNote)

using namespace csapex;

CoreNote::CoreNote()
{
}

CoreNote::CoreNote(CoreNoteType request_type) : NoteImplementation(AUUID::NONE), note_type_(request_type)
{
}

CoreNote::CoreNote(CoreNoteType request_type, const std::vector<std::any>& payload) : NoteImplementation(AUUID::NONE), note_type_(request_type), payload_(payload)
{
}

void CoreNote::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Note::serialize(data, version);

    data << note_type_;
    data << payload_;
}

void CoreNote::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Note::deserialize(data, version);

    data >> note_type_;
    data >> payload_;
}
