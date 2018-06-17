/// HEADER
#include <csapex/io/protcol/profiler_note.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/note_serializer.h>

CSAPEX_REGISTER_NOTE_SERIALIZER(ProfilerNote)

using namespace csapex;

ProfilerNote::ProfilerNote()
{
}

ProfilerNote::ProfilerNote(ProfilerNoteType request_type, const AUUID &uuid)
    : NoteImplementation(uuid),
      note_type_(request_type)
{

}

ProfilerNote::ProfilerNote(ProfilerNoteType request_type, const AUUID &uuid, const std::vector<boost::any> &payload)
    : NoteImplementation(uuid),
      note_type_(request_type),
      payload_(payload)
{

}

void ProfilerNote::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    Note::serialize(data, version);

    data << note_type_;
    data << payload_;
}

void ProfilerNote::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Note::deserialize(data, version);

    data >> note_type_;
    data >> payload_;
}
