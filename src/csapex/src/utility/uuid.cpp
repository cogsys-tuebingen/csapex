/// HEADER
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <stdexcept>
#include <iostream>

using namespace csapex;

UUID UUID::NONE("");
const std::string UUID::namespace_separator = ":|:";


namespace {
void split_first(const std::string& haystack, const std::string& needle,
                 /* OUTPUTS: */ std::string& lhs, std::string& rhs)
{
    size_t pos = haystack.find(needle);
    if(pos == haystack.npos) {
        return;
    }

    lhs = haystack.substr(0, pos);
    rhs = haystack.substr(pos + needle.length());
}

}

std::map<std::string, int> UUID::hash_;

void UUID::reset()
{
    hash_.clear();
}

std::string UUID::stripNamespace(const std::string &name)
{
    size_t from = name.rfind("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}

UUID UUID::make(const std::string &prefix)
{
    // ensure uniqueness
    std::string rep = prefix;
    if(hash_.find(rep) != hash_.end()) {
        throw std::runtime_error("the UUID " + prefix + " is already taken");
    }

    UUID r(rep);
    hash_[r.representation_]++;
    return r;
}

UUID UUID::make_sub(const UUID &parent, const std::string &prefix)
{
    return make(parent.getFullName() + UUID::namespace_separator + prefix);
}

void UUID::free(const UUID &uuid)
{
    std::map<std::string, int>::iterator it = hash_.find(uuid.representation_);
    if(it != hash_.end()) {
        hash_.erase(it);
    }
}

UUID UUID::make_forced(const std::string &representation)
{
    UUID r(representation);
    return r;
}

UUID::UUID(const std::string &representation)
{
    representation_ = representation;
}

UUID::operator std::string() const
{
    return representation_;
}

const char* UUID::c_str() const
{
    return representation_.c_str();
}

std::string UUID::getFullName() const
{
    return representation_;
}

std::string UUID::getShortName() const
{
    return stripNamespace(representation_);
}

bool UUID::contains(const std::string &sub) const
{
    size_t pos = representation_.find(sub);
    return pos != representation_.npos;
}

UUID UUID::replace(const std::string &needle, const UUID &replacement) const
{
    size_t pos = representation_.find(needle);
    std::string tmp = representation_;
    tmp.replace(pos, needle.length(), replacement);
    return UUID::make(tmp);
}

void UUID::split(const std::string &separator, UUID &l, UUID &r) const
{
    split_first(representation_, separator, l.representation_, r.representation_);
}
