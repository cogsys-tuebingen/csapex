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
#include <algorithm>

using namespace csapex;

const std::string UUID::namespace_separator = ":|:";
UUID UUID::NONE;
AUUID AUUID::NONE;

std::size_t UUID::Hasher::operator()(const UUID& k) const
{
    return k.hash();
}

bool UUID::empty() const
{
    return representation_.empty();
}
std::size_t UUID::depth() const
{
    return representation_.size();
}

bool UUID::global() const
{
    if (empty() || composite()) {
        return false;
    }

    return representation_.back().at(0) == ':';
}

std::string UUID::globalName() const
{
    apex_assert_hard(global());
    return representation_.back().substr(1);
}

std::string UUID::stripNamespace(const std::string& name)
{
    size_t from = name.rfind("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}

UUID::UUID()
{
}

UUID::UUID(const UUID& other) : parent_(other.parent_), representation_(other.representation_)
{
}

UUID::~UUID()
{
}

UUID& UUID::operator=(const UUID& other)
{
    parent_ = other.parent_;
    representation_ = other.representation_;
    return *this;
}

UUID::UUID(std::weak_ptr<UUIDProvider> parent, const UUID& copy) : UUID(parent, copy.representation_)
{
}

UUID::UUID(std::weak_ptr<UUIDProvider> parent, const std::vector<std::string>& representation) : parent_(parent), representation_(representation)
{
    apex_assert_hard(representation_.empty() || representation_.back() != "~");
}

UUID::UUID(std::weak_ptr<UUIDProvider> parent, const std::string& representation) : parent_(parent)
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
    while (end >= 0) {
        std::size_t pos = representation.rfind(namespace_separator, end - 1);
        int begin = 0;
        if (pos != std::string::npos) {
            begin = pos + namespace_separator.length();
        }

        std::string sub_id = representation.substr(begin, end - begin);

        if (sub_id != "~") {
            representation_.push_back(sub_id);
        }
        end = pos;

        if (begin == 0) {
            return;
        }
    }
    apex_assert_hard(representation_.empty() || representation_.back() != "~");
}

void UUID::free()
{
    if (auto parent = parent_.lock()) {
        parent->free(*this);
    }
}

bool UUID::operator<(const UUID& other) const
{
    return representation_ < other.representation_;
}

std::string UUID::getFullName() const
{
    if (empty()) {
        return "~";
    }

    std::stringstream ss;
    auto it = representation_.rbegin();
    ss << *it;
    for (++it; it != representation_.rend(); ++it) {
        ss << namespace_separator;
        ss << *it;
    }
    return ss.str();
}

std::size_t UUID::hash() const
{
    if (empty()) {
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

bool UUID::contains(const std::string& sub) const
{
    for (const std::string& s : representation_) {
        if (s == sub) {
            return true;
        }
    }
    return false;
}

UUID UUID::parentUUID() const
{
    UUID parent = *this;
    if (!representation_.empty()) {
        parent.representation_.erase(parent.representation_.begin());
    }

    return parent;
}

UUID UUID::nestedUUID() const
{
    return reshape(depth() - 1);
}
UUID UUID::rootUUID() const
{
    if (auto parent = parent_.lock()) {
        return UUID(parent, representation_.back());
    } else {
        return UUID(std::weak_ptr<UUIDProvider>(), representation_.back());
    }
}

UUID UUID::reshape(std::size_t _depth) const
{
    if (_depth > depth()) {
        throw std::invalid_argument("cannot reshape UUID to a larger size");
    }
    return UUID(parent_, std::vector<std::string>(representation_.begin(), representation_.begin() + static_cast<long>(_depth)));
}
UUID UUID::reshapeSoft(std::size_t max_depth) const
{
    return UUID(parent_, std::vector<std::string>(representation_.begin(), representation_.begin() + static_cast<long>(std::min(depth(), max_depth))));
}

UUID UUID::makeRelativeTo(const UUID& prefix) const
{
    auto prefix_it = prefix.representation_.rbegin();
    auto this_it = representation_.rbegin();
    while (prefix_it != prefix.representation_.rend() && this_it != representation_.rend() && *this_it == *prefix_it) {
        ++prefix_it;
        ++this_it;
    }

    auto reversed = std::vector<std::string>(this_it, representation_.rend());
    std::reverse(reversed.begin(), reversed.end());
    return UUID(parent_, reversed);
}

UUID UUID::id() const
{
    return reshape(1);
}

std::string UUID::type() const
{
    apex_assert_hard(!representation_.empty());
    std::string t = representation_.front();
    return t.substr(0, t.find("_"));
}
std::string UUID::name() const
{
    apex_assert_hard(!representation_.empty());
    std::string t = representation_.front();
    return t.substr(t.find("_") + 1);
}

AUUID UUID::getAbsoluteUUID() const
{
    if (auto parent = parent_.lock()) {
        UUID parent_uuid = parent->getAbsoluteUUID();
        UUID uuid = *this;
        for (const std::string& part : parent_uuid.representation_) {
            uuid.representation_.push_back(part);
        }
        return AUUID(uuid);
    } else {
        return AUUID(*this);
    }
}

bool UUID::hasParent() const
{
    return parent_.lock() != nullptr;
}

std::shared_ptr<UUIDProvider> UUID::getParent() const
{
    return parent_.lock();
}

namespace csapex
{
bool operator==(const std::string& str, const UUID& uuid_)
{
    return str == uuid_.getFullName();
}
bool operator==(const UUID& uuid_, const std::string& str)
{
    return str == uuid_.getFullName();
}
bool operator==(const UUID& a, const UUID& b)
{
    if (a.representation_.size() != b.representation_.size()) {
        return false;
    }

    for (auto ita = a.representation_.begin(), itb = b.representation_.begin(); ita != a.representation_.end(); ++ita, ++itb) {
        if (*ita != *itb) {
            return false;
        }
    }

    return true;
}

bool operator!=(const UUID& a, const UUID& b)
{
    return !(a == b);
}

std::ostream& operator<<(std::ostream& out, const UUID& uuid_)
{
    out << uuid_.getFullName();
    return out;
}
}  // namespace csapex

/**
 * AUUID
 */

AUUID::AUUID(const UUID& uuid) : UUID(uuid)
{
}
AUUID& AUUID::operator=(const UUID& uuid)
{
    UUID::operator=(uuid);
    return *this;
}

std::size_t AUUID::Hasher::operator()(const AUUID& k) const
{
    return k.hash();
}

AUUID AUUID::parentAUUID() const
{
    AUUID parent = *this;
    if (!representation_.empty()) {
        parent.representation_.erase(parent.representation_.begin());
    }

    return parent;
}

AUUID AUUID::getAbsoluteUUID() const
{
    return *this;
}

namespace csapex
{
std::ostream& operator<<(std::ostream& out, const AUUID& uuid_)
{
    out << "*" << uuid_.getFullName() << "*";
    return out;
}
}  // namespace csapex

bool AUUID::operator<(const AUUID& other) const
{
    return UUID::operator<(other);
}
