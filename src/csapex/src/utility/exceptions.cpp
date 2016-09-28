/// HEADER
#include <csapex/utility/exceptions.h>

/// COMPONENT
#include <csapex/utility/thread.h>

/// SYSTEM
#include <iostream>
#include <sstream>
#ifdef WIN32

#else
#include <execinfo.h>
#include <cxxabi.h>
#endif


using namespace csapex;

Failure::Failure()
    : stack_depth_(0)
{
#ifdef WIN32
#else
    void *stack_addrs[max_depth];
    stack_depth_ = backtrace(stack_addrs, max_depth);
    char **stack_strings = backtrace_symbols(stack_addrs, stack_depth_);

    stack_strings_.reserve(stack_depth_);
    int line = 0;
    for (size_t i = 0; i < stack_depth_; i++) {
        //stack_strings_[i] = stack_strings[i];
        std::stringstream ss;

        char *mangled_name = 0, *offset_begin = 0, *offset_end = 0;

        // find parantheses and +address offset surrounding mangled name
        for (char *p = stack_strings[i]; *p; ++p)
        {
            if (*p == '(')
            {
                mangled_name = p;
            }
            else if (*p == '+')
            {
                offset_begin = p;
            }
            else if (*p == ')')
            {
                offset_end = p;
                break;
            }
        }

        // if the line could be processed, attempt to demangle the symbol
        if (mangled_name && offset_begin && offset_end &&
                mangled_name < offset_begin)
        {
            *mangled_name++ = '\0';
            *offset_begin++ = '\0';
            *offset_end++ = '\0';

            int status;
            char * real_name_c = abi::__cxa_demangle(mangled_name, 0, 0, &status);

            // if demangling is successful, output the demangled function name
            if (status == 0)
            {
                std::string real_name = real_name_c;
                if((real_name.size() >= 15 && (real_name.substr(0, 15) == "csapex::Failure")) ||
                        (real_name.size() >= 28 && (real_name.substr(0, 28) == "csapex::HardAssertionFailure")) ||
                        (real_name.size() >= 12 && (real_name.substr(0, 12) == "_apex_assert"))) {
                    continue;
                }

                ss << "(" << line++ << ") " << stack_strings[i] << " : "
                          << real_name << "+" << offset_begin << offset_end;

            }
            // otherwise, output the mangled function name
            else
            {
                ss << "(" << line++ << ") " << stack_strings[i] << " : "
                          << mangled_name << "+" << offset_begin << offset_end;
            }
            free(real_name_c);
        }
        // otherwise, print the whole line
        else
        {
            ss << "(" << line++ << ") " << stack_strings[i];
        }

        stack_strings_.push_back(ss.str());
    }

    stack_depth_ = line;

    free(stack_strings); // malloc()ed by backtrace_symbols
#endif
}

Failure::Failure(const char *m)
    : Failure()
{
    msg = m;
}

Failure::Failure(const std::string &m)
    : Failure()
{
    msg = m;
}

Failure::~Failure()
{

}

void Failure::printStackTrace() const
{
    std::cerr << what() << std::endl;
}

std::string Failure::type() const
{
    return "Fatal Error";
}


std::ostream &Failure::stackTrace(std::ostream &ss, std::size_t depth) const
{
    if(!msg.empty()) {
        ss << msg << "\n\n";
    }

    reason(ss);

    ss << "Stack trace:\n";

    size_t i = 1;
    for (; i < stack_depth_ && i < depth; i++) {
        ss << "    " << stack_strings_[i] << '\n';
    }

    if(i != stack_depth_) {
        ss << "... " << (stack_depth_ - i) << " levels";
    }

    return ss;
}

std::ostream &Failure::reason(std::ostream &ss) const
{
    ss << "A serious error happened:" << std::endl;
    return ss;
}



std::string Failure::what() const
{
    std::stringstream ss;
    stackTrace(ss);
    return ss.str();
}

Failure* Failure::clone() const
{
    return new Failure(*this);
}


HardAssertionFailure::HardAssertionFailure()
    : line(-1)
{

}

HardAssertionFailure::HardAssertionFailure(const char *msg, const char* code, const char* file, int line, const char *signature)
    : Failure(msg), code(code), file(file), line(line), signature(signature), thread(csapex::thread::get_name())
{
}

HardAssertionFailure::~HardAssertionFailure()
{
}

std::string HardAssertionFailure::what() const
{
    std::stringstream ss;
    if(!msg.empty()) {
        ss << msg << " - ";
    }

    stackTrace(ss, max_depth);

    return ss.str();
}


std::ostream &HardAssertionFailure::reason(std::ostream& ss) const
{
    ss << "Assertion \"" << code << "\" failed in " << file << ", line " << line << ", function: " << signature << ", thread \"" << csapex::thread::get_name() << "\"" << std::endl;
    return ss;
}


std::string HardAssertionFailure::type() const
{
    return "Assertion Failure";
}

Failure* HardAssertionFailure::clone() const
{
    return new HardAssertionFailure(*this);
}
