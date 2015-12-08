#ifndef EXECPTIONS_H
#define EXECPTIONS_H

#include <exception>
#include <string>
#include <vector>

namespace csapex
{

class HardAssertionFailure
{
public:
    HardAssertionFailure();
    HardAssertionFailure(const char* code, const char* file, int line);
    ~HardAssertionFailure();

    std::string what() const;
    std::string stackTrace(std::size_t depth = max_depth) const;

    void printStackTrace() const;

public:
    std::string code;
    std::string file;
    int line;

    std::string thread;

private:
    static constexpr size_t max_depth = 100;

    size_t stack_depth_;
    std::vector<std::string> stack_strings_;
};

}

#endif // EXECPTIONS_H
