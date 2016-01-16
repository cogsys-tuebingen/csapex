#ifndef UUID_H
#define UUID_H

/// SYSTEM
#include <string>
#include <map>
#include <mutex>

namespace csapex {

class UUIDProvider;

class UUID
{
    friend class UUIDProvider;

public:
    static std::string stripNamespace(const std::string& name);

    static UUID NONE;
    static const std::string namespace_separator;


public:
    friend std::ostream& operator << (std::ostream& out, const UUID& uuid_);

    friend bool operator == (const std::string& str, const UUID& uuid_);
    friend bool operator == (const UUID& uuid_, const std::string& str);
    friend bool operator == (const UUID& a, const UUID& b);

    bool operator < (const UUID& rhs) const;

    struct Hasher {
      std::size_t operator()(const UUID& k) const;
    };


public:
    UUID();

    void free();

    std::string getFullName() const;
    std::string getShortName() const;

    std::size_t hash() const;

    bool contains(const std::string& sub) const;

    UUID parentUUID() const;
    std::string type() const;
    std::string id() const;

    bool empty() const;

private:
    explicit UUID(UUIDProvider *parent, const std::string& representation);

    void split(const std::string& separator, UUID& l, UUID& r) const;

private:
    UUIDProvider* parent_;
    std::string representation_;
};

}
#endif // UUID_H
