#ifndef UUID_H
#define UUID_H

/// SYSTEM
#include <string>
#include <ostream>

namespace csapex {

class UUID
{
    friend class Unique;

public:
    static std::string stripNamespace(const std::string& name);

    friend std::ostream& operator << (std::ostream& out, const UUID& uuid_) {
        out << uuid_.representation_;
        return out;
    }

    friend bool operator == (const std::string& str, const UUID& uuid_) {
        return str == uuid_.representation_;
    }

    friend bool operator == (const UUID& uuid_, const std::string& str) {
        return str == uuid_.representation_;
    }

public:
    std::string getFullName() const;
    std::string getShortName() const;

    operator std::string() const;
    const char* c_str() const;

    bool empty() const {
        return representation_.empty();
    }

private:
    UUID(const std::string& representation);

private:
    std::string representation_;
};

}
#endif // UUID_H
