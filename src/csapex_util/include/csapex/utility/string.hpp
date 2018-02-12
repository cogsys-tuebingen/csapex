#ifndef STRING_HPP
#define STRING_HPP

/// PROJECT
#include <csapex/utility/type.h>

/// SYSTEM
#include <string>
#include <memory>
#include <type_traits>
#include <vector>

namespace csapex
{

inline std::string universal_to_string(const bool value)
{
    return value ? "true" : "false";
}
inline std::string universal_to_string(const std::string& string)
{
    return string;
}

inline std::string universal_to_string(const char* string)
{
    return std::string(string);
}
template <typename V>
inline std::string universal_to_string(const V& value,
                                       typename std::enable_if<std::is_arithmetic<V>::value>::type* = 0)
{
    return std::to_string(value);
}

template <typename V>
inline std::string universal_to_string(const std::vector<V>& value)
{
    return std::string("[vector of length ") + std::to_string(value.size()) + "]";
}
template <typename V>
inline std::string universal_to_string(const std::pair<V,V>& value)
{
    return std::string("(") + universal_to_string(value.first) + ", " + universal_to_string(value.second) + ")";
}

template <typename V>
inline std::string universal_to_string(const V* value)
{
    return std::string("<") + csapex::type2name(typeid(V)) + "* = " + std::to_string((long) value) + ">";
}

template <typename V>
inline std::string universal_to_string(const V& value,
                                       typename std::enable_if<!std::is_arithmetic<V>::value>::type* = 0)
{
    return std::string("<") + csapex::type2name(typeid(V)) + "& = " + std::to_string((long) &value) + ">";
}


template <typename V>
inline std::string universal_to_string(const std::shared_ptr<V>& value)
{
    return universal_to_string(value.get());
}


}

#endif // STRING_HPP
