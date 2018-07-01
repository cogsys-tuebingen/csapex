#ifndef DATA_TRAITS_HPP
#define DATA_TRAITS_HPP

/// COMPONENT
#include <csapex/utility/tmp.hpp>
#include <csapex/utility/semantic_version.h>

/// SYSTEM
#include <memory>

namespace csapex
{
HAS_MEM_FUNC(makeEmpty, has_make_empty);

template <typename T, typename std::enable_if<has_make_empty<T, std::shared_ptr<T> (*)()>::value, int>::type = 0>
inline std::shared_ptr<T> makeEmpty()
{
    return T::makeEmpty();
}

template <typename T, typename std::enable_if<!has_make_empty<T, std::shared_ptr<T> (*)()>::value, int>::type = 0>
inline std::shared_ptr<T> makeEmpty()
{
    return std::make_shared<T>();
}

// semantic version of token data
template <typename T>
struct semantic_version
{
    // default
    static constexpr SemanticVersion value{ 0, 0, 0 };
};

}  // namespace csapex

#endif  // DATA_TRAITS_HPP
