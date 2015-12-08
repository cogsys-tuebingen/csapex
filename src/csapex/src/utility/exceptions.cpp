/// HEADER
#include <csapex/utility/exceptions.h>

/// COMPONENT
#include <csapex/utility/thread.h>

/// SYSTEM
#include <execinfo.h>
#include <iostream>
#include <sstream>

using namespace csapex;

HardAssertionFailure::HardAssertionFailure()
    : line(-1), stack_depth_(0)
{

}

HardAssertionFailure::HardAssertionFailure(const char* code, const char* file, int line)
    : code(code), file(file), line(line), thread(csapex::thread::get_name())
{
    void *stack_addrs[max_depth];

    stack_depth_ = backtrace(stack_addrs, max_depth);
    char **stack_strings = backtrace_symbols(stack_addrs, stack_depth_);

    stack_strings_.resize(stack_depth_);
    for (size_t i = 0; i < stack_depth_; i++) {
        stack_strings_[i] = stack_strings[i];
    }


    free(stack_strings); // malloc()ed by backtrace_symbols
}

HardAssertionFailure::~HardAssertionFailure()
{
}

void HardAssertionFailure::printStackTrace() const
{
    std::cerr << what() << std::endl;
}

std::string HardAssertionFailure::what() const
{
    std::stringstream ss;
    ss << "assertion \"" << code << "\" failed in " << file << ", line " << line << ", thread \"" << csapex::thread::get_name() << "\"" << std::endl;

    ss << stackTrace();

    return ss.str();
}

std::string HardAssertionFailure::stackTrace(std::size_t depth) const
{
    std::stringstream ss;
    ss << "Call stack from " <<  file << " : " << line << '\n';

    size_t i = 1;
    for (; i < stack_depth_ && i < depth; i++) {
        ss << "    " << stack_strings_[i] << '\n';
    }

    if(i != stack_depth_) {
        ss << "... " << (stack_depth_ - i) << " levels";
    }

    return ss.str();
}
