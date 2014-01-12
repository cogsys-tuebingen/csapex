/// HEADER
#include <csapex/utility/uuid.h>

using namespace csapex;

UUID UUID::NONE("");


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

std::string UUID::stripNamespace(const std::string &name)
{
    size_t from = name.rfind("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}

UUID UUID::make(const std::string &prefix)
{
    // TODO: ensure uniqueness
    return UUID(prefix);
}

UUID UUID::make_forced(const std::string &representation)
{
    // DON'T ensure uniqueness
    return UUID(representation);
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
