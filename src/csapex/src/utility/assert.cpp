/// HEADER
#include <csapex/utility/assert.h>

/// PROJECT
#include <csapex/utility/thread.h>
#include <csapex/utility/exceptions.h>

/// SYSTEM
#include <stdexcept>
#include <sstream>
#include <iostream>

void _apex_assert(bool assertion, const char* code, const char* file, int line)
{
    if(!assertion) {
        std::stringstream ss;
        ss << "[cs::APEX - ASSERTION FAILED] \"" << code << "\" [file " << file << ", line " << line << ", thread \"" << csapex::thread::get_name() << "\"]";
        throw std::logic_error(ss.str());
    }
}

void _apex_assert_hard(bool assertion, const char* code, const char* file, int line)
{
    if(!assertion) {
        throw csapex::HardAssertionFailure(code, file, line);
    }
}

void _apex_assert_soft(bool assertion, const char* code, const char* file, int line)
{
    if(!assertion) {
        std::cerr << "[cs::APEX - SOFT ASSERTION FAILED] \"" << code << "\" [file " << file << ", line " << line << "]" << std::endl;
    }
}
