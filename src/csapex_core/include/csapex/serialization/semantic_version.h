#ifndef SEMANTIC_VERSION_H
#define SEMANTIC_VERSION_H

namespace csapex
{

/**
 * @brief The SemanticVersion struct represents a semantic version
 *
 * Given a version number MAJOR.MINOR.PATCH, increment the:
 *  MAJOR version when you make incompatible API changes,
 *  MINOR version when you add functionality in a backwards-compatible manner, and
 *  PATCH version when you make backwards-compatible bug fixes.
 *
 * @see https://semver.org/
 */
struct SemanticVersion
{
    int major_v = 0;
    int minor_v = 0;
    int patch_v = 0;

    SemanticVersion(int major, int minor, int patch);

    SemanticVersion() = default;

    bool operator != (const SemanticVersion& other);
    bool operator == (const SemanticVersion& other);

    bool operator < (const SemanticVersion& other);
    bool operator <= (const SemanticVersion& other);

    bool operator > (const SemanticVersion& other);
    bool operator >= (const SemanticVersion& other);


    bool valid() const;
    operator bool() const;
};

}

#endif // SEMANTIC_VERSION_H
