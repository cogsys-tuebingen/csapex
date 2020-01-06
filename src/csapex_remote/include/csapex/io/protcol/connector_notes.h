#ifndef CONNECTOR_NOTES_H
#define CONNECTOR_NOTES_H

/// PROJECT
#include <csapex/io/note_impl.hpp>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <any>

namespace csapex
{
enum class ConnectorNoteType
{
/**
 * begin: connect signals
 **/
#define HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_STATIC_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) function##Changed,
#include <csapex/model/connector_proxy_accessors.hpp>
    /**
     * end: connect signals
     **/
};

class ConnectorNote : public NoteImplementation<ConnectorNote>
{
public:
    ConnectorNote();
    ConnectorNote(ConnectorNoteType request_type, const AUUID& uuid);
    ConnectorNote(ConnectorNoteType request_type, const AUUID& uuid, const std::any& payload);

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    ConnectorNoteType getNoteType() const
    {
        return note_type_;
    }

    std::any getPayload() const
    {
        return payload_;
    }

private:
    ConnectorNoteType note_type_;
    std::any payload_;
};

}  // namespace csapex

#endif  // CONNECTOR_NOTES_H
