#ifndef TAG_H
#define TAG_H

/// COMPONENT
#include <csapex_core/csapex_core_export.h>
#include <csapex/serialization/streamable.h>

/// SYSTEM
#include <string>
#include <map>
#include <memory>

namespace csapex
{

class CSAPEX_CORE_EXPORT Tag : public Streamable
{
protected:
    CLONABLE_IMPLEMENTATION(Tag);

public:
    typedef std::shared_ptr<Tag> Ptr;

    static const uint8_t PACKET_TYPE_ID = 129;

private:
    class Manager {
    public:
        static Manager& instance() {
            static Manager inst;
            return inst;
        }

    public:
        const Tag::Ptr get(const std::string& name);
        bool exists(const std::string& name) const;
        void create(const std::string &name);

    private:
        Manager();

        std::map<std::string, Tag::Ptr> tags_;
    };

public:
    ~Tag();

    static const Tag::Ptr get(const std::string& name);
    static bool exists(const std::string& name);
    static void create(const std::string& name);
    static void createIfNotExists(const std::string& name);

    std::string getName() const;

    int compare (const Tag& tag) const;
    bool operator < (const Tag& tag) const;

    virtual uint8_t getPacketType() const;

    virtual void serialize(SerializationBuffer &data, SemanticVersion& version) const;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version);

    static Ptr makeEmpty();

private:
    Tag(const std::string& name);
    Tag();

private:
    std::string name_;
};

}

#endif // TAG_H
