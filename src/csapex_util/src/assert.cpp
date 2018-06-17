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

#ifdef WIN32
# if defined (__MINGW32__)
#  define TRIGGER_BREAKPOINT() DebugBreak();
# else // MSVC
#  define TRIGGER_BREAKPOINT() __debugbreak();
# endif
#elif defined(__powerpc64__)
# define TRIGGER_BREAKPOINT() asm volatile ("tw 31,1,1");
#elif defined(__i386__) || defined(__ia64__) || defined(__x86_64__)
# include <signal.h>
# define TRIGGER_BREAKPOINT() raise(SIGTRAP);
#else
# include <stdlib.h>
# define TRIGGER_BREAKPOINT() abort();
#endif

void _apex_fail(const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig)
{
    std::stringstream ss;
    ss << "[cs::APEX - ASSERTION FAILED] ";
    if(!msg.empty()) {
        ss << msg << " ";
    }
    if(!code.empty()) {
        ss << "\"" << code << "\"";
    }
    ss << "[file " << file << ", line " << line << ", function: " << sig << ", thread \"" << csapex::thread::get_name() << "\"]";
    throw std::logic_error(ss.str());
}

void _apex_assert(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig)
{
    if(!assertion) {
        TRIGGER_BREAKPOINT();
        _apex_fail(msg, code, file, line, sig);
    }
}

void _apex_assert_hard(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig)
{
    if(!assertion) {
        TRIGGER_BREAKPOINT();
        throw csapex::HardAssertionFailure(msg, code, file, line, sig);
    }
}

void _apex_assert_soft(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig)
{
    if(!assertion) {
        std::cerr << "[cs::APEX - SOFT ASSERTION FAILED] ";
        if(!msg.empty()) {
            std::cerr << msg << " ";
        }
        std::cerr << "\"" << code << "\" [file " << file << ", line " << line << ", function: " << sig << ", thread \"" << csapex::thread::get_name() << "]" << std::endl;

        TRIGGER_BREAKPOINT();
    }
}
