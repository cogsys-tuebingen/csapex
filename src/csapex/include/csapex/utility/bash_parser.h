#ifndef BASE_PARSER_H
#define BASE_PARSER_H

/// PROJECT
#include <csapex/csapex_util_export.h>

/// SYSTEM
#include <string>
#include <sstream>

namespace csapex
{

class CSAPEX_UTILS_EXPORT BashParser
{
public:
    BashParser();

    static std::string toHtml(const std::string& input);

private:
    static std::string set(int code);
    static std::string color(int code);
};

}

#endif // BASE_PARSER_H
