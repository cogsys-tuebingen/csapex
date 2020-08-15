#ifndef GRAPH_NOTES_H
#define GRAPH_NOTES_H

/// PROJECT
#include <csapex/io/note_impl.hpp>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/any.h>

namespace csapex
{
enum class GraphNoteType
{
    ConnectionAdded,
    ConnectionRemoved,

    VertexAdded,
    VertexRemoved,

/**
 * begin: connect signals
 **/
#define HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_STATIC_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) function##Changed,
#define HANDLE_SIGNAL(_enum, signal) _enum##Triggered,
#include <csapex/model/graph_proxy_accessors.hpp>
    /**
     * end: connect signals
     **/
};

class GraphNote : public NoteImplementation<GraphNote>
{
public:
    GraphNote();
    GraphNote(GraphNoteType request_type, const AUUID& uuid);
    GraphNote(GraphNoteType request_type, const AUUID& uuid, const std::vector<std::any>& payload);

    template <typename... Args>
    GraphNote(GraphNoteType request_type, const AUUID& uuid, Args... args) : GraphNote(request_type, uuid, { std::forward<Args>(args)... })
    {
    }

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    GraphNoteType getNoteType() const
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
        return std::any_cast<T>(payload_.at(pos));
    }

private:
    GraphNoteType note_type_;
    std::vector<std::any> payload_;
};

}  // namespace csapex

#endif  // GRAPH_NOTES_H
