#ifndef TYPE_H
#define TYPE_H

/// PROJECT
#include <csapex/csapex_util_export.h>

/// SYSTEM
#include <string>

namespace csapex
{
CSAPEX_UTILS_EXPORT std::string type2name(const std::type_info& info);
CSAPEX_UTILS_EXPORT std::string type2nameWithoutNamespace(const std::type_info& info);
}

#endif // TYPE_H
