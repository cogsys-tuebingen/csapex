/// HEADER
#include <csapex/utility/type.h>

/// SYSTEM
#include <string>
#include <cxxabi.h>

namespace {
std::string replace(const std::string& s, const std::string& find, const std::string& replace) {
    std::string result = s;
    std::size_t ptr_pos = s.find(find);
    if(ptr_pos != std::string::npos) {
        result.replace(ptr_pos, find.size(), replace);
    }
    return result;
}
}

std::string csapex::type2name(const std::type_info& info)
{
    int status;
    std::string full_name(abi::__cxa_demangle(info.name(), 0, 0, &status));

    return full_name;
}

std::string csapex::type2nameWithoutNamespace(const std::type_info& info)
{
    int status;
    std::string full_name(abi::__cxa_demangle(info.name(), 0, 0, &status));

    //    return replace(replace(full_name, "connection_types::", ""), "csapex::", "");
    std::size_t split_point = full_name.rfind("::");

    if(split_point == full_name.npos) {
        return full_name;
    } else {
        return full_name.substr(split_point+2);
    }
}

