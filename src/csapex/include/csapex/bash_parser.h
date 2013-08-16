#ifndef BASE_PARSER_H
#define BASE_PARSER_H

/// SYSTEM
#include <string>
#include <sstream>

namespace csapex
{

class BashParser
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
