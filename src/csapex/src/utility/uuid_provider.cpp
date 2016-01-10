/// HEADER
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <sstream>

using namespace csapex;

UUIDProvider::UUIDProvider()
{

}

UUIDProvider::~UUIDProvider()
{
    reset();
}


void UUIDProvider::reset()
{
    std::unique_lock<std::mutex> lock(hash_mutex_);
    hash_.clear();

    uuids_.clear();
}

UUID UUIDProvider::makeUUID(const std::string &name)
{
    std::unique_lock<std::mutex> lock(hash_mutex_);

    // ensure uniqueness
    std::string rep = name;
    if(hash_.find(rep) != hash_.end()) {
        throw std::runtime_error("the UUID " + name + " is already taken");
    }

    UUID r(this, rep);
    hash_[r.representation_]++;
    return r;
}

UUID UUIDProvider::generateUUID(const std::string &prefix)
{
    return makeUUID(generateUniquePrefix(prefix));
}

UUID UUIDProvider::makeDerivedUUID(const UUID &parent, const std::string &name)
{
    return makeUUID(parent.getFullName() + UUID::namespace_separator + name);
}

UUID UUIDProvider::makeDerivedUUID_forced(const UUID &parent, const std::string &name)
{
    return makeUUID_forced( parent.getFullName() + UUID::namespace_separator + name);
}

void UUIDProvider::free(const UUID &uuid)
{
    std::unique_lock<std::mutex> lock(hash_mutex_);

    std::map<std::string, int>::iterator it = hash_.find(uuid.representation_);
    if(it != hash_.end()) {
        hash_.erase(it);
    }
}

UUID UUIDProvider::makeUUID_forced(const std::string &representation)
{
    UUID r(nullptr, representation);
    return r;
}

UUID UUIDProvider::makeConnectableUUID(const UUID &parent, const std::string& type, int sub_id)
{
    if(parent.empty()) {
        return UUID::NONE;
    }

    std::stringstream ss;
    ss << parent << UUID::namespace_separator << type << "_" << sub_id;
    return makeUUID(ss.str());
}

UUID UUIDProvider::makeConnectableUUID_forced(const UUID &parent, const std::string& type, int sub_id)
{
    if(parent.empty()) {
        return UUID::NONE;
    }

    std::stringstream ss;
    ss << parent << UUID::namespace_separator << type << "_" << sub_id;
    return makeUUID_forced(ss.str());
}


std::string UUIDProvider::generateUniquePrefix(const std::string& name)
{
    int& last_id = uuids_[name];
    ++last_id;

    std::stringstream ss;
    ss << name << "_" << last_id;

    return ss.str();
}
