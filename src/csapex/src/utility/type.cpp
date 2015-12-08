/// HEADER
#include <csapex/utility/type.h>

/// SYSTEM
#include <string>
#include <cxxabi.h>
#include <cstdlib>

std::string csapex::type2name(const std::type_info& info)
{
    int status;
    char* demangled = abi::__cxa_demangle(info.name(), 0, 0, &status);
    std::string full_name(demangled);
    free(demangled);

    return full_name;
}

std::string csapex::type2nameWithoutNamespace(const std::type_info& info)
{
    int status;
    char* demangled = abi::__cxa_demangle(info.name(), 0, 0, &status);
    std::string full_name(demangled);
    free(demangled);

    //    return replace(replace(full_name, "connection_types::", ""), "csapex::", "");
    std::size_t split_point = full_name.find("::");

    if(split_point == full_name.npos) {
        return full_name;
    } else {
        return full_name.substr(split_point+2);
    }
}

