#ifndef DATA_TRAITS_HPP
#define DATA_TRAITS_HPP

/// COMPONENT
#include <csapex/utility/tmp.hpp>

/// SYSTEM
#include <memory>

namespace csapex
{

HAS_MEM_FUNC(makeEmpty, has_make_empty);

template <typename T, typename std::enable_if<has_make_empty<T, std::shared_ptr<T>(*)()>::value, int>::type = 0>
inline std::shared_ptr<T> makeEmpty()
{
    return T::makeEmpty();
}

template <typename T, typename std::enable_if<!has_make_empty<T, std::shared_ptr<T>(*)()>::value, int>::type = 0>
inline std::shared_ptr<T> makeEmpty()
{
    return std::make_shared<T>();
}

}

#endif // DATA_TRAITS_HPP
