/// HEADAER
#include <csapex/utility/semantic_version.h>

/// SYSTEM
#include <vector>
#include <sstream>
#include <iostream>

using namespace csapex;

SemanticVersion::SemanticVersion(const std::string& version_str)
{
    char delimiter = '.';
    std::vector<int> versions;
    auto start_pos = 0;
    auto end_pos = version_str.size();
    while(start_pos < end_pos) {
        auto it = version_str.find(delimiter, start_pos);
        if(it > end_pos) {
            it = end_pos;
        }
        std::string token = version_str.substr(start_pos, it-start_pos);
        versions.push_back(std::atoi(token.c_str()));
        start_pos = it+1;
    }
    while(versions.size() < 3) {
        versions.push_back(0);
    }
    major_v = versions[0];
    minor_v = versions[1];
    patch_v = versions[2];
}

std::string SemanticVersion::toString() const
{
    std::stringstream ss;
    ss << major_v << '.' << minor_v << '.' << patch_v;
    return ss.str();
}

bool SemanticVersion::operator<(const SemanticVersion& other)
{
    if (major_v < other.major_v) {
        return true;
    } else if (major_v > other.major_v) {
        return false;
    }
    // major equal
    if (minor_v < other.minor_v) {
        return true;
    } else if (minor_v > other.minor_v) {
        return false;
    }
    // major and minor equal
    return patch_v < other.patch_v;
}

bool SemanticVersion::operator==(const SemanticVersion& other)
{
    return major_v == other.major_v && minor_v == other.minor_v && patch_v == other.patch_v;
}

bool SemanticVersion::operator>(const SemanticVersion& other)
{
    return (operator>=(other)) && (operator!=(other));
}

bool SemanticVersion::operator>=(const SemanticVersion& other)
{
    return !(operator<(other));
}
bool SemanticVersion::operator<=(const SemanticVersion& other)
{
    return (operator<(other)) || (operator==(other));
}

bool SemanticVersion::operator!=(const SemanticVersion& other)
{
    return !(operator==(other));
}

bool SemanticVersion::valid() const
{
    return major_v >= 0;
}

SemanticVersion::operator bool() const
{
    return valid();
}
