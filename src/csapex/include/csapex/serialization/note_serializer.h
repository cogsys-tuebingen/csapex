#ifndef NOTE_SERIALIZER_H
#define NOTE_SERIALIZER_H

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/io/note.h>

/// SYSTEM
#include <inttypes.h>

namespace csapex
{

class NoteSerializerInterface
{
public:
    virtual ~NoteSerializerInterface();

    virtual void serialize(const io::NoteConstPtr& packet, SerializationBuffer &data) = 0;
    virtual io::NotePtr deserialize(const SerializationBuffer& data) = 0;
};


class NoteSerializer : public Singleton<NoteSerializer>, public Serializer
{
public:
    void serialize(const StreamableConstPtr& packet, SerializationBuffer &data) override;
    StreamablePtr deserialize(const SerializationBuffer &data) override;

    static void registerSerializer(const std::string& type, std::shared_ptr<NoteSerializerInterface> serializer);

private:
    std::map<std::string, std::shared_ptr<NoteSerializerInterface>> serializers_;
};


/// REGISTRATION

template <typename S>
struct NoteSerializerRegistered
{
    NoteSerializerRegistered(const std::string& type) {
        NoteSerializer::registerSerializer(type, std::make_shared<S>());
    }
};
}


#define CSAPEX_REGISTER_NOTE_SERIALIZER(Name) \
    namespace csapex \
    { \
    namespace io \
    { \
    class Name##Serializer : public NoteSerializerInterface \
    { \
        virtual void serialize(const io::NoteConstPtr& packet, SerializationBuffer &data) override \
        { \
            packet->serialize(data); \
        } \
        virtual io::NotePtr deserialize(const SerializationBuffer& data) override \
        { \
            auto result = std::make_shared<Name>(); \
            result->deserialize(data); \
            return result; \
        } \
    }; \
    } \
    NoteSerializerRegistered<io::Name##Serializer> g_register_note_##Name##_(Name::typeName()); \
    }


#endif // NOTE_SERIALIZER_H
