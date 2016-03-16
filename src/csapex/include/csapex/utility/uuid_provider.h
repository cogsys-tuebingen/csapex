#ifndef UUID_PROVIDER_H
#define UUID_PROVIDER_H

/// COMPONENT
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{

class UUIDProvider
{
    friend class UUID;
    friend class GraphIO; // TODO: remove

public:
    UUIDProvider(UUIDProvider* parent = nullptr, AUUID auuid = AUUID());
    virtual ~UUIDProvider();

    void setParent(UUIDProvider* parent, AUUID auuid);

    UUID makeUUID(const std::string& name);
    UUID generateUUID(const std::string& prefix);

    UUID makeDerivedUUID(const UUID& parent, const std::string& name);
    UUID generateDerivedUUID(const UUID& parent, const std::string& name);

    UUID makeTypedUUID(const UUID &parent, const std::string &type, int sub_id);
    UUID makeTypedUUID(const UUID &parent, const std::string &type, const std::string& sub_id);
    UUID generateTypedUUID(const UUID &parent, const std::string &type);

    static UUID makeUUID_forced(const std::string& representation);
    static UUID makeDerivedUUID_forced(const UUID& parent, const std::string& name);
    static UUID makeTypedUUID_forced(const UUID &parent, const std::string &type, int sub_id);
    static UUID makeTypedUUID_forced(const UUID &parent, const std::string &type, const std::string& sub_id);


    void registerUUID(const UUID& uuid);
    bool exists(const UUID& uuid);

    std::map<std::string, int> getUUIDMap() const;

    AUUID getAbsoluteUUID() const;

protected:
    void reset();
    void free(const UUID& uuid);
    std::string generateNextName(const std::string& name);
    std::string generateNextSubName(const UUID& parent, const std::string& name);

protected:
    UUIDProvider* parent_provider_;
    AUUID auuid_;

    std::recursive_mutex hash_mutex_;
    std::map<std::string, int> hash_;

    std::map<std::string, int> uuids_;
    std::unordered_map<UUID, std::map<std::string, int>, UUID::Hasher> sub_uuids_;
};

}

#endif // UUID_PROVIDER_H
