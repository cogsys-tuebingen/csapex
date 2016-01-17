/// HEADER
#include <csapex/utility/uuid.h>

/// COMPONENT
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <stdexcept>
#include <iostream>
#include <boost/functional/hash.hpp>
#include <ostream>

using namespace csapex;

UUID UUID::NONE(nullptr, "");
const std::string UUID::namespace_separator = ":|:";


namespace {
void split_first(const std::string& haystack, const std::string& needle,
                 /* OUTPUTS: */ std::string& lhs, std::string& rhs)
{
    size_t pos = haystack.find(needle);
    if(pos == haystack.npos) {
        lhs = haystack;
        return;
    }

    lhs = haystack.substr(0, pos);
    rhs = haystack.substr(pos + needle.length());
}

}

std::size_t UUID::Hasher::operator()(const UUID& k) const {
    return k.hash();
}

bool UUID::empty() const
{
    return representation_ == "invalid_uuid";
}

std::string UUID::stripNamespace(const std::string &name)
{
    size_t from = name.rfind("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}


UUID::UUID()
    : parent_(nullptr), representation_("invalid_uuid")
{
}

UUID::UUID(UUIDProvider* parent, const std::string &representation)
    : parent_(parent), representation_(representation)
{
}

void UUID::free()
{
    if(parent_) {
        parent_->free(*this);
    }
}

bool UUID::operator <(const UUID& other) const
{
    return representation_ < other.representation_;
}

std::string UUID::getFullName() const
{
    return representation_;
}

std::size_t UUID::hash() const
{
    return boost::hash<std::string>()(representation_);
}

std::string UUID::getShortName() const
{
    return stripNamespace(representation_);
}

bool UUID::composite() const
{
    return contains(UUID::namespace_separator);
}

bool UUID::contains(const std::string &sub) const
{
    size_t pos = representation_.find(sub);
    return pos != representation_.npos;
}

UUID UUID::parentUUID() const
{
    UUID l = UUID::NONE;
    UUID r = UUID::NONE;
    split(UUID::namespace_separator, l, r);

    return l;
}

UUID UUID::firstChildUUID() const
{
    UUID l = UUID::NONE;
    UUID r = UUID::NONE;
    split(UUID::namespace_separator, l, r);

    return r;
}

std::string UUID::id() const
{
    return firstChildUUID().getFullName();
}

std::string UUID::type() const
{
    std::string t = id();
    return t.substr(0, t.find_first_of("_"));
}

void UUID::split(const std::string &separator, UUID &l, UUID &r) const
{
    split_first(representation_, separator, l.representation_, r.representation_);
}

namespace csapex
{
bool operator == (const std::string& str, const UUID& uuid_) {
    return str == uuid_.representation_;
}
bool operator == (const UUID& uuid_, const std::string& str) {
    return str == uuid_.representation_;
}
bool operator == (const UUID& a, const UUID& b) {
    return a.representation_ == b.representation_;
}

bool operator != (const UUID& a, const UUID& b) {
    return !(a == b);
}

std::ostream& operator << (std::ostream& out, const UUID& uuid_) {
    out << uuid_.representation_;
    return out;
}
}
