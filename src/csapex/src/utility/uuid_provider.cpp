/// HEADER
#include <csapex/utility/uuid_provider.h>

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <sstream>
#include <iostream>

using namespace csapex;

UUIDProvider::UUIDProvider()
{

}

UUIDProvider::~UUIDProvider()
{
    clearCache();
}

void UUIDProvider::setParent(std::weak_ptr<UUIDProvider> parent, AUUID auuid)
{
    parent_provider_ = parent;
    auuid_ = auuid;
}

void UUIDProvider::clearCache()
{
    std::unique_lock<std::recursive_mutex > lock(hash_mutex_);
    hash_.clear();

    uuids_.clear();
    sub_uuids_.clear();
}

UUID UUIDProvider::makeUUID(const std::string &name)
{
    std::unique_lock<std::recursive_mutex > lock(hash_mutex_);

    // ensure uniqueness
    std::string rep = name;
    if(hash_.find(rep) != hash_.end()) {
        throw std::runtime_error("the UUID " + name + " is already taken");
    }

    UUID r(shared_from_this(), rep);
    registerUUID(r);
    return r;
}

void UUIDProvider::registerUUID(const UUID& id)
{
    std::unique_lock<std::recursive_mutex > lock(hash_mutex_);
    apex_assert_hard(!id.representation_.empty());
    hash_[id.getFullName()]++;
}

bool UUIDProvider::exists(const UUID &uuid)
{
     return hash_.find(uuid.getFullName()) != hash_.end();
}

UUID UUIDProvider::generateUUID(const std::string &prefix)
{
    std::unique_lock<std::recursive_mutex > lock(hash_mutex_);

    UUID r;
    // ensure uniqueness
    do {
        std::string name = generateNextName(prefix);
        r = UUID(shared_from_this(), name);

    } while(exists(r));

    registerUUID(r);
    return r;
}

UUID UUIDProvider::makeDerivedUUID(const UUID &parent, const std::string &name)
{
    return makeUUID(parent.getFullName() + UUID::namespace_separator + name);
}

UUID UUIDProvider::generateDerivedUUID(const UUID &parent, const std::string &prefix)
{
    //return generateUUID(parent.getFullName() + UUID::namespace_separator + name);

    std::unique_lock<std::recursive_mutex > lock(hash_mutex_);

    UUID r;
    do {
        std::string name = generateNextSubName(parent, prefix);
        r = UUID (shared_from_this(), parent.getFullName() + UUID::namespace_separator + name);
    } while(exists(r));

    registerUUID(r);
    return r;

}

UUID UUIDProvider::makeDerivedUUID_forced(const UUID &parent, const std::string &name)
{
    return makeUUID_forced(parent.parent_, parent.getFullName()  + UUID::namespace_separator + name);
}

void UUIDProvider::free(const UUID &uuid)
{
    std::unique_lock<std::recursive_mutex > lock(hash_mutex_);

    apex_assert_hard(!uuid.representation_.empty());;
    std::map<std::string, int>::iterator it = hash_.find(uuid.getFullName());
    if(it != hash_.end()) {
        hash_.erase(it);
    }

    auto sub_it = sub_uuids_.find(uuid.parentUUID());
    if(sub_it != sub_uuids_.end()) {
        sub_uuids_.erase(sub_it);
    }
}

UUID UUIDProvider::makeUUID_without_parent(const std::string &representation)
{
    return UUID(std::weak_ptr<UUIDProvider>(), representation);
}

UUID UUIDProvider::makeUUID_forced(std::weak_ptr<UUIDProvider> parent, const std::string &representation)
{
    return UUID (parent, representation);
}

UUID UUIDProvider::generateTypedUUID(const UUID &parent, const std::string& type)
{
    if(parent.empty()) {
        return UUID::NONE;
    }

    return generateDerivedUUID(parent, type);
}

UUID UUIDProvider::makeTypedUUID(const UUID &parent, const std::string& type, int sub_id)
{
    return makeTypedUUID(parent, type, std::to_string(sub_id));
}

UUID UUIDProvider::makeTypedUUID(const UUID &parent, const std::string& type, const std::string& sub_id)
{
    if(parent.empty()) {
        return UUID::NONE;
    }

    return makeDerivedUUID(parent, type + "_" + sub_id);
}

UUID UUIDProvider::makeTypedUUID_forced(const UUID &parent, const std::string& type, int sub_id)
{
    return makeTypedUUID_forced(parent, type, std::to_string(sub_id));
}

UUID UUIDProvider::makeTypedUUID_forced(const UUID &parent, const std::string& type, const std::string& sub_id)
{
    if(parent.empty()) {
        return UUID::NONE;
    }

    return makeDerivedUUID_forced(parent, type + "_" + sub_id);
}


std::string UUIDProvider::generateNextName(const std::string& name)
{
    int& next_id = uuids_[name];

    std::stringstream ss;
    ss << name << "_" << next_id;

    ++next_id;
    return ss.str();
}

std::string UUIDProvider::generateNextSubName(const UUID& parent, const std::string& name)
{
    int& next_id = sub_uuids_[parent][name];

    std::stringstream ss;
    ss << name << "_" << next_id;

    ++next_id;
    return ss.str();
}


std::map<std::string, int> UUIDProvider::getUUIDMap() const
{
    return uuids_;
}

AUUID UUIDProvider::getAbsoluteUUID() const
{
    return auuid_;
}
