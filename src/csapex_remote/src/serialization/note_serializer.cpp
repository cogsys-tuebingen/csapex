/// HEADER
#include <csapex/serialization/note_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <iostream>

using namespace csapex;
using namespace csapex::io;

SerializerRegistered<NoteSerializer> g_register_note_serializer_(Note::PACKET_TYPE_ID, &NoteSerializer::instance());


NoteSerializerInterface::~NoteSerializerInterface()
{

}

void NoteSerializer::serialize(const Streamable& packet, SerializationBuffer& data)
{
    if(const Note* note = dynamic_cast<const Note*>(&packet)) {
//        std::cerr << "serializing Note" << std::endl;
        std::string type = note->getType();
        auto it = serializers_.find(type);
        if(it != serializers_.end()) {

//            std::cerr << "serializing Note (type=" << type << ")" << std::endl;
            data << type;

            // defer serialization to the corresponding serializer
            std::shared_ptr<NoteSerializerInterface> serializer = it->second;
            serializer->serialize(*note, data);

        } else {
            std::cerr << "cannot serialize Note of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }

    }
}

StreamablePtr NoteSerializer::deserialize(const SerializationBuffer& data)
{
//    std::cerr << "deserializing Note" << std::endl;

    std::string type;
    data >> type;

    auto it = serializers_.find(type);
    if(it != serializers_.end()) {

//        std::cerr << "deserializing Note (type=" << type << ")" << std::endl;

        // defer serialization to the corresponding serializer
        std::shared_ptr<NoteSerializerInterface> serializer = it->second;
        return serializer->deserialize(data);

    } else {
        std::cerr << "cannot deserialize Note of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
    }


    return NotePtr();
}

void NoteSerializer::registerSerializer(const std::string &type, std::shared_ptr<NoteSerializerInterface> serializer)
{
//    std::cout << "registering serializer of type " << type << std::endl;
    instance().serializers_[type] = serializer;
}
