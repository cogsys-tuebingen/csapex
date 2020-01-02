#ifndef STRING_HPP
#define STRING_HPP

/// PROJECT
#include <csapex/utility/type.h>

/// SYSTEM
#include <sstream>
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
inline std::string universal_to_string(const V& value, typename std::enable_if<std::is_arithmetic<V>::value>::type* = 0)
{
    return std::to_string(value);
}

template <typename V>
inline std::string universal_to_string(const std::vector<V>& value)
{
    return std::string("[vector of length ") + std::to_string(value.size()) + "]";
}
template <typename V>
inline std::string universal_to_string(const std::pair<V, V>& value)
{
    return std::string("(") + universal_to_string(value.first) + ", " + universal_to_string(value.second) + ")";
}

template <typename V>
inline std::string universal_to_string(const V* value)
{
    std::stringstream ss;
    ss << "<" << csapex::type2name(typeid(V)) << "* = " << static_cast<const void*>(value) << ">";
    return ss.str();
}

template <typename V>
inline std::string universal_to_string(const V& value, typename std::enable_if<!std::is_arithmetic<V>::value>::type* = 0)
{
    std::stringstream ss;
    ss << "<" << csapex::type2name(typeid(V)) << "& = " << static_cast<const void*>(&value) << ">";
    return ss.str();
}

template <typename V>
inline std::string universal_to_string(const std::shared_ptr<V>& /* value */)
{
    return universal_to_string(false);
}

}  // namespace csapex

#endif  // STRING_HPP
