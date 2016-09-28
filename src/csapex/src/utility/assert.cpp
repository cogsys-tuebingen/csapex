/// HEADER
#include <csapex/utility/assert.h>

/// PROJECT
#include <csapex/utility/thread.h>
#include <csapex/utility/exceptions.h>

/// SYSTEM
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <cstring>

void _apex_assert(bool assertion, const char* msg, const char* code, const char* file, int line, const char* sig)
{
    if(!assertion) {
        std::stringstream ss;
        ss << "[cs::APEX - ASSERTION FAILED] ";
        if(strlen(msg) > 0) {
            ss << msg << " ";
        }
        ss << "\"" << code << "\" [file " << file << ", line " << line << ", function: " << sig << ", thread \"" << csapex::thread::get_name() << "\"]";
        throw std::logic_error(ss.str());
    }
}

void _apex_assert_hard(bool assertion, const char* msg, const char* code, const char* file, int line, const char* sig)
{
    if(!assertion) {
        throw csapex::HardAssertionFailure(msg, code, file, line, sig);
    }
}

void _apex_assert_soft(bool assertion, const char* msg, const char* code, const char* file, int line, const char *sig)
{
    if(!assertion) {
        std::cerr << "[cs::APEX - SOFT ASSERTION FAILED] ";
        if(strlen(msg) > 0) {
            std::cerr << msg << " ";
        }
        std::cerr << "\"" << code << "\" [file " << file << ", line " << line << ", function: " << sig << ", thread \"" << csapex::thread::get_name() << "]" << std::endl;
    }
}
