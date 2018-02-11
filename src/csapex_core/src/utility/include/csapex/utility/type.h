#ifndef TYPE_H
#define TYPE_H

/// PROJECT
#include <csapex_util_export.h>

/// SYSTEM
#include <string>
#include <typeinfo>

namespace csapex
{

/**
 * @brief type2name convertes a type into is demangled C++ name
 * @param info typeinfo of the type to convert
 * @return the converted type
 */
CSAPEX_UTILS_EXPORT
std::string type2name(const std::type_info& info);

/**
 * @brief type2name convertes a type is demangled C++ name
 * @tparam T type of the type to convert
 * @return the converted type
 */
template <typename T>
CSAPEX_UTILS_EXPORT
std::string type2name()
{
    return type2name(typeid(T));
}

/**
 * @brief type2nameWithoutNamespace convertes a type into is demangled C++ name, dropping all namespaces
 * @param info typeinfo of the type to convert
 * @return the converted type
 */
CSAPEX_UTILS_EXPORT
std::string type2nameWithoutNamespace(const std::type_info& info);

/**
 * @brief type2nameWithoutNamespace convertes a type is demangled C++ name, dropping all namespaces
 * @tparam T type of the type to convert
 * @return the converted type
 */
template <typename T>
CSAPEX_UTILS_EXPORT
std::string type2nameWithoutNamespace()
{
    return type2nameWithoutNamespace(typeid(T));
}

}

#endif // TYPE_H
