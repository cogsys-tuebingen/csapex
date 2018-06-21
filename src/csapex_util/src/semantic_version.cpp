/// HEADAER
#include <csapex/utility/semantic_version.h>

using namespace csapex;


bool SemanticVersion::operator < (const SemanticVersion& other)
{
    if(major_v < other.major_v) {
        return true;
    } else if(major_v > other.major_v) {
        return false;
    }
    // major equal
    if(minor_v < other.minor_v) {
        return true;
    } else if(minor_v > other.minor_v) {
        return false;
    }
    // major and minor equal
    return patch_v < other.patch_v;
}

bool SemanticVersion::operator == (const SemanticVersion& other)
{
    return major_v == other.major_v &&
            minor_v == other.minor_v &&
            patch_v == other.patch_v;
}


bool SemanticVersion::operator > (const SemanticVersion& other)
{
    return (operator >= (other)) && (operator != (other));
}

bool SemanticVersion::operator >= (const SemanticVersion& other)
{
    return !(operator < (other));
}
bool SemanticVersion::operator <= (const SemanticVersion& other)
{
    return (operator < (other)) || (operator == (other));
}

bool SemanticVersion::operator != (const SemanticVersion& other)
{
    return !(operator == (other));
}

bool SemanticVersion::valid() const
{
    return major_v >= 0;
}

SemanticVersion::operator bool() const
{
    return valid();
}

