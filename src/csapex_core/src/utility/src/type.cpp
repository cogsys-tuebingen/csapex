/// HEADER
#include <csapex/utility/type.h>

/// SYSTEM
#include <string>
#include <cstdlib>

#ifdef WIN32
#else
#include <cxxabi.h>
#endif


std::string csapex::type2name(const std::type_info& info)
{
#ifdef WIN32
	return info.name();
#else
    int status;
    char* demangled = abi::__cxa_demangle(info.name(), 0, 0, &status);
    std::string full_name(demangled);
    free(demangled);

    return full_name;
#endif
}

std::string csapex::type2nameWithoutNamespace(const std::type_info& info)
{
	std::string full_name;
#ifdef WIN32
	full_name = info.name();
#else
    int status;
    char* demangled = abi::__cxa_demangle(info.name(), 0, 0, &status);
    full_name = demangled;
    free(demangled);
#endif

    //    return replace(replace(full_name, "connection_types::", ""), "csapex::", "");
    std::size_t split_point = full_name.find("::");

    if(split_point == full_name.npos) {
        return full_name;
    } else {
        return full_name.substr(split_point+2);
    }
}

