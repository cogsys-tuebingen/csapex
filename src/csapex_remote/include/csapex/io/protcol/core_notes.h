#ifndef CORE_NOTES_H
#define CORE_NOTES_H

/// PROJECT
#include <csapex/io/note_impl.hpp>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{
enum class CoreNoteType
{
    ConfigChanged,

    StepBegin,
    StepEnd,

    StatusChanged,
    NewNodeType,
    NewSnippetType,
    ResetRequested,
    ResetDone,
    Saved,
    Loaded,
    Paused,
    SteppingEnabled,

    //    SaveDetailRequest,
    //    LoadDetailRequest,

    Notification
};

class CoreNote : public NoteImplementation<CoreNote>
{
public:
    CoreNote();
    CoreNote(CoreNoteType request_type);
    CoreNote(CoreNoteType request_type, const std::vector<boost::any>& payload);

    template <typename... Args>
    CoreNote(CoreNoteType request_type, Args... args) : CoreNote(request_type, { std::forward<Args>(args)... })
    {
    }

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    CoreNoteType getNoteType() const
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
    CoreNoteType note_type_;
    std::vector<boost::any> payload_;
};

}  // namespace csapex

#endif  // CORE_NOTES_H
