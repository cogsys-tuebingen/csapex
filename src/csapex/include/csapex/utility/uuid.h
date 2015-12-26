#ifndef UUID_H
#define UUID_H

/// SYSTEM
#include <string>
#include <map>
#include <mutex>

namespace csapex {

class UUID
{
public:
    static std::string stripNamespace(const std::string& name);
    static UUID make(const std::string& prefix);
    static UUID make_forced(const std::string& representation);

    static UUID make_sub(const UUID& parent, const std::string& prefix);
    static UUID make_sub_forced(const UUID& parent, const std::string& prefix);

    static void free(const UUID& uuid);

    static void reset();

    static UUID NONE;
    static const std::string namespace_separator;

public:
    friend std::ostream& operator << (std::ostream& out, const UUID& uuid_);

    friend bool operator == (const std::string& str, const UUID& uuid_);
    friend bool operator == (const UUID& uuid_, const std::string& str);
    friend bool operator == (const UUID& a, const UUID& b);

    struct Hasher {
      std::size_t operator()(const UUID& k) const;
    };


public:
    UUID();

    std::string getFullName() const;
    std::string getShortName() const;

    std::size_t hash() const;

    bool contains(const std::string& sub) const;
    UUID replace(const std::string& needle, const UUID& replacement) const;

    UUID parentUUID() const;
    std::string type() const;


    operator std::string() const;
    const char* c_str() const;

    bool empty() const;

private:
    explicit UUID(const std::string& representation);

    void split(const std::string& separator, UUID& l, UUID& r) const;

private:
    std::string representation_;

    static std::mutex hash_mutex_;
    static std::map<std::string, int> hash_;
};

}
#endif // UUID_H
