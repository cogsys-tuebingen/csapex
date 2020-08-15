#ifndef CSAPEX_UTIL_ANY_H
#define CSAPEX_UTIL_ANY_H

#define ANY_EXPERIMENTAL 0

#if defined(__clang__)
#elif defined(__GNUC__) || defined(__GNUG__)
#include <features.h>
#if __GNUC_PREREQ(7, 0)
#else
#undef ANY_EXPERIMENTAL
#define ANY_EXPERIMENTAL 1
#endif
#endif

#if ANY_EXPERIMENTAL
#include <experimental/any>

namespace std
{
using experimental::any;
using experimental::any_cast;
using experimental::bad_any_cast;
}  // namespace std

static bool any_has_value(const std::any& a)
{
    return !a.empty();
}
#else
#include <any>

static bool any_has_value(const std::any& a)
{
    return a.has_value();
}
#endif

#endif  // CSAPEX_UTIL_ANY_H