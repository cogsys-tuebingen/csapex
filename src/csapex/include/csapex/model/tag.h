#ifndef TAG_H
#define TAG_H

/// COMPONENT
#include <csapex/csapex_export.h>

/// SYSTEM
#include <string>
#include <map>
#include <memory>

namespace csapex
{

class CSAPEX_EXPORT Tag
{
public:
    typedef std::shared_ptr<Tag> Ptr;

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

private:
    Tag(const std::string& name);

private:
    std::string name_;
};

}

#endif // TAG_H
