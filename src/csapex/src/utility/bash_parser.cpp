/// HEADER
#include <csapex/utility/bash_parser.h>

/// SYSTEM
#include <boost/regex.hpp>
#include <sstream>

using namespace csapex;

BashParser::BashParser()
{
}

std::string BashParser::toHtml(const std::string &input)
{
    std::string result = input;

    static const boost::regex bash_fmt(".*(\e\\[[^m]+m).*");

    int levels = 0;
    boost::match_results<std::string::const_iterator> results;
    while (boost::regex_match(result, results, bash_fmt)) {
        std::string bash_string = results[1];

        std::stringstream ss(bash_string.substr(2,bash_string.size()-3));
        std::string token;
        std::stringstream replacement;
        while (std::getline(ss, token, ';')) {
            int code = atoi(token.c_str());
            if(code == 0) {
                for(int l = 0; l < levels; ++l) {
                    replacement << "</span>";
                }
                levels = 0;
            } else {
                levels++;

                if(1 <= code && code <= 28) {
                    replacement << set(code);
                } else if(30 <= code && code <= 107) {
                    replacement << color(code);
                } else {
                    replacement << "<span>" << code ;
                }

            }
        }

        result.replace(results.position(1), bash_string.size(), replacement.str());
    }

    return result;
}

std::string BashParser::set(int code)
{
    std::string res = "<span style='";
    switch(code) {
    case 1:
        res += "font-weight: bold";
        break;
    case 21:
        res += "font-weight: normal";
        break;

    case 2:
        res += "font-weight: lighter";
        break;
    case 22:
        res += "font-weight: normal";
        break;

    case 3:
        res += "text-decoration:underline";
        break;
    case 23:
        res += "text-decoration:none";
        break;
    }

    return res + "'>";
}

std::string BashParser::color(int code)
{
    std::string res = "<span style='";
    switch(code) {
    case 39:
    case 30:
        res += "color: black";
        break;
    case 31:
        res += "color: red";
        break;
    case 32:
        res += "color: green";
        break;
    case 33:
        res += "color: yellow";
        break;
    case 34:
        res += "color: blue";
        break;
    case 35:
        res += "color: magenta";
        break;
    case 36:
        res += "color: cyan";
        break;
    case 37:
        res += "color: lightgray";
        break;
    case 90:
        res += "color: darkgray";
        break;
    case 91:
        res += "color: lightred";
        break;
    case 92:
        res += "color: lightgreen";
        break;
    case 93:
        res += "color: lightyellow";
        break;
    case 94:
        res += "color: lightblue";
        break;
    case 95:
        res += "color: lightmagenta";
        break;
    case 96:
        res += "color: lightcyan";
        break;
    case 97:
        res += "color: white";
        break;
    }

    return res + "'>";
}
