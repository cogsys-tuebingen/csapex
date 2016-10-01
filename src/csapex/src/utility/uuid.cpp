/// HEADER
#include <csapex/utility/uuid.h>

/// COMPONENT
#include <csapex/utility/uuid_provider.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <stdexcept>
#include <iostream>
#include <boost/functional/hash.hpp>
#include <ostream>
#include <sstream>

using namespace csapex;

const std::string UUID::namespace_separator = ":|:";
UUID UUID::NONE(std::weak_ptr<UUIDProvider>(), "");

std::size_t UUID::Hasher::operator()(const UUID& k) const {
    return k.hash();
}

bool UUID::empty() const
{
    return representation_.empty();
}

std::string UUID::stripNamespace(const std::string &name)
{
    size_t from = name.rfind("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}


UUID::UUID()
{
}

UUID::UUID(std::weak_ptr<UUIDProvider> parent, const std::vector<std::string> &representation)
    : parent_(parent), representation_(representation)
{
    apex_assert_hard(representation_.empty() || representation_.back() != "~");
}

UUID::UUID(std::weak_ptr<UUIDProvider> parent, const std::string &representation)
    : parent_(parent)
{
    /**
     *  UUIDs are built like this:
     *
     *   <-----> :|: <-------> :|: <--------->
     *
     *  Invariants while splitting:
     *                         pos
     *                         |
     *   <-----> :|: <-------> :|: <--------->
     *                             |         |
     *                             begin     end
     *
     *  End condition:
     *   pos
     *   |
     *   <-----> :|: <-------> :|: <--------->
     *   |     |
     *  begin  end
     */

    int end = representation.length();
    while(end >= 0) {
        std::size_t pos = representation.rfind(namespace_separator, end-1);
        int begin = 0;
        if(pos != std::string::npos) {
            begin = pos + namespace_separator.length();
        }

        std::string sub_id = representation.substr(begin, end - begin);

        if(sub_id != "~") {
            representation_.push_back(sub_id);
        }
        end = pos;

        if(begin == 0) {
            return;
        }

    }
    apex_assert_hard(representation_.empty() || representation_.back() != "~");
}

void UUID::free()
{
    if(auto parent = parent_.lock()) {
        parent->free(*this);
    }
}

bool UUID::operator <(const UUID& other) const
{
    return representation_ < other.representation_;
}

std::string UUID::getFullName() const
{
    if(empty()) {
        return "~";
    }


    std::stringstream ss;
    auto it = representation_.rbegin();
    ss << *it;
    for(++it; it != representation_.rend(); ++it) {
        ss << namespace_separator;
        ss << *it;
    }
    return ss.str();
}

std::size_t UUID::hash() const
{
    if(empty()) {
        return 0;
    } else {
        return boost::hash<std::string>()(representation_.front());
    }
}

std::string UUID::getShortName() const
{
    return stripNamespace(representation_.front());
}

bool UUID::composite() const
{
    return representation_.size() > 1;
}

bool UUID::contains(const std::string &sub) const
{
    for(const std::string& s : representation_) {
        if(s == sub) {
            return true;
        }
    }
    return false;
}

UUID UUID::parentUUID() const
{
    UUID parent = *this;
    if(!representation_.empty()) {
        parent.representation_.erase(parent.representation_.begin());
    }

    return parent;
}

UUID UUID::nestedUUID() const
{
    UUID parent = *this;
    if(!representation_.empty()) {
        parent.representation_.erase(--parent.representation_.end());
    }

    return parent;
}
UUID UUID::rootUUID() const
{
    if(auto parent = parent_.lock()) {
        return UUID(parent, representation_.back());
    } else {
        return UUID(std::weak_ptr<UUIDProvider>(), representation_.back());
    }
}

UUID UUID::id() const
{
    UUID res = *this;
    res.representation_.clear();
    res.representation_.push_back(representation_.front());
    return res;
}

std::string UUID::type() const
{
    std::string t = representation_.front();
    return t.substr(0, t.find("_"));
}
std::string UUID::name() const
{
    std::string t = representation_.front();
    return t.substr(t.find("_") + 1);
}

AUUID UUID::getAbsoluteUUID() const
{
    if(auto parent = parent_.lock()) {
//        return AUUID(parent->makeDerivedUUID_forced(parent->getAbsoluteUUID(), id()));

        UUID parent_uuid = parent->getAbsoluteUUID();
        UUID uuid = *this;
        for(const std::string& part : parent_uuid.representation_) {
            uuid.representation_.push_back(part);
        }
        return AUUID(uuid);
    } else {
        return AUUID(*this);
    }
}

namespace csapex
{
bool operator == (const std::string& str, const UUID& uuid_) {
    return str == uuid_.getFullName();
}
bool operator == (const UUID& uuid_, const std::string& str) {
    return str == uuid_.getFullName();
}
bool operator == (const UUID& a, const UUID& b) {
    if(a.representation_.size() != b.representation_.size()) {
        return false;
    }

    for(auto ita = a.representation_.begin(), itb = b.representation_.begin();
        ita != a.representation_.end(); ++ita, ++itb) {
        if(*ita != *itb) {
            return false;
        }
    }

    return true;
}

bool operator != (const UUID& a, const UUID& b) {
    return !(a == b);
}

std::ostream& operator << (std::ostream& out, const UUID& uuid_) {
    out << uuid_.getFullName();
    return out;
}
}


/**
 * AUUID
 */

AUUID::AUUID(const UUID &uuid)
    : UUID(uuid)
{

}
AUUID& AUUID::operator = (const UUID& uuid)
{
    UUID::operator = (uuid);
    return *this;
}

std::size_t AUUID::Hasher::operator()(const AUUID& k) const {
    return k.hash();
}

AUUID AUUID::parentAUUID() const
{
    AUUID parent = *this;
    if(!representation_.empty()) {
        parent.representation_.erase(parent.representation_.begin());
    }

    return parent;
}

namespace csapex
{

std::ostream& operator << (std::ostream& out, const AUUID& uuid_) {
    out << "*" << uuid_.getFullName() << "*";
    return out;
}
}

bool AUUID::operator <(const AUUID& other) const
{
    return UUID::operator <(other);
}
