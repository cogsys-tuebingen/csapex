/// HEADER
#include <csapex/utility/assert.h>

/// SYSTEM
#include <assert.h>
#include <stdexcept>
#include <sstream>
#include <iostream>

void _apex_assert(bool assertion, const char* code, const char* file, int line)
{
    _apex_assert_hard(assertion, code, file, line);
}

void _apex_assert_hard(bool assertion, const char* code, const char* file, int line)
{
    if(!assertion) {
        std::stringstream ss;
        ss << "assertion \"" << code << "\" failed in " << file << ", line " << line ;
        throw std::runtime_error(ss.str());
    }
}

void _apex_assert_soft(bool assertion, const char* code, const char* file, int line)
{
    if(!assertion) {
        std::cerr << "[cs::APEX - SOFT ASSERTION FAILED] \"" << code << "\" [file " << file << ", line " << line << "]" << std::endl;
    }
}
