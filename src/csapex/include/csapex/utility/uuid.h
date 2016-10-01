#ifndef UUID_H
#define UUID_H

/// PROJECT
#include <csapex/csapex_util_export.h>

/// SYSTEM
#include <string>
#include <map>
#include <vector>
#include <memory>

namespace csapex {

class UUIDProvider;
class AUUID;

/**
 * @brief The UUID class represents unique IDs
 *
 * The uniqueness is guaranteed in the whole graph structure recursively:
 * Every UUID consists of multiple layers for each sub graph:
 * Let ID be a vector of individual identifiers
 *  - ID[0]    - the unique id of this instance
 *  - ID[1]    - the unique id of the parent id
 *  - ...
 */
class CSAPEX_UTILS_EXPORT UUID
{
    friend class UUIDProvider;

public:
	static std::string stripNamespace(const std::string& name);

	static const std::string namespace_separator;

	static UUID NONE;

public:
    friend bool CSAPEX_UTILS_EXPORT operator == (const std::string& str, const UUID& uuid_);
    friend bool CSAPEX_UTILS_EXPORT operator == (const UUID& uuid_, const std::string& str);

    friend bool CSAPEX_UTILS_EXPORT operator == (const UUID& a, const UUID& b);
    friend bool CSAPEX_UTILS_EXPORT operator != (const UUID& a, const UUID& b);

    bool operator < (const UUID& rhs) const;

	struct CSAPEX_UTILS_EXPORT Hasher {
      std::size_t operator()(const UUID& k) const;
    };


public:
    UUID();

    void free();

    std::string getFullName() const;
    std::string getShortName() const;

    std::size_t hash() const;

    bool composite() const;
    UUID nestedUUID() const;
    UUID rootUUID() const;

    bool contains(const std::string& sub) const;

    UUID parentUUID() const;
    UUID id() const;

    std::string name() const;
    std::string type() const;

    bool empty() const;

    AUUID getAbsoluteUUID() const;

private:
    explicit UUID(std::weak_ptr<UUIDProvider> parent, const std::string& representation);
    explicit UUID(std::weak_ptr<UUIDProvider> parent, const std::vector<std::string>& representation);

protected:
    std::weak_ptr<UUIDProvider> parent_;
    std::vector<std::string> representation_;
};

/**
 * @brief The AUUID class represents an *absolute* UUID
 */
class CSAPEX_UTILS_EXPORT AUUID : public UUID
{
public:
    AUUID() = default;
    AUUID(const AUUID& uuid) = default;
    explicit AUUID(const UUID& uuid);

    AUUID parentAUUID() const;

    AUUID& operator = (const AUUID& uuid) = default;
    AUUID& operator = (const UUID& uuid);


    bool operator < (const AUUID& rhs) const;

    struct Hasher {
      std::size_t operator()(const AUUID& k) const;
    };

};


CSAPEX_UTILS_EXPORT std::ostream& operator << (std::ostream& out, const UUID& uuid_);

}
#endif // UUID_H
