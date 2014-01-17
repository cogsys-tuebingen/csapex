#ifndef UUID_H
#define UUID_H

/// SYSTEM
#include <string>
#include <ostream>
#include <map>

namespace csapex {

class UUID
{
public:
    static std::string stripNamespace(const std::string& name);
    static UUID make(const std::string& prefix);
    static UUID make_forced(const std::string& representation);

    static void reset();

    static UUID NONE;

public:
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
    friend bool operator == (const UUID& a, const UUID& b) {
        return a.representation_ == b.representation_;
    }

public:
    std::string getFullName() const;
    std::string getShortName() const;

    bool contains(const std::string& sub) const;
    UUID replace(const std::string& needle, const UUID& replacement) const;

    void split(const std::string& separator, UUID& l, UUID& r) const;

    operator std::string() const;
    const char* c_str() const;

    bool empty() const {
        return representation_.empty();
    }

private:
    explicit UUID(const std::string& representation);

private:
    std::string representation_;

    static std::map<std::string, int> hash_;
};

}
#endif // UUID_H
