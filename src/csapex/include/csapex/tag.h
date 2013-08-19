#ifndef TAG_H
#define TAG_H

/// SYSTEM
#include <string>
#include <map>

namespace csapex
{

class Tag
{
private:
    class Manager {
    public:
        static Manager& instance();
        const Tag get(const std::string& name) const;
        bool exists(const std::string& name) const;
        void create(const std::string &name);

    private:
        Manager();

        std::map<std::string, Tag> tags_;
    };


public:
    ~Tag();

    static const Tag get(const std::string& name);
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
