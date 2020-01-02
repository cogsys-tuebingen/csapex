#ifndef EXECPTIONS_H
#define EXECPTIONS_H

/// PROJECT
#include <csapex_util/export.h>

/// SYSTEM
#include <exception>
#include <string>
#include <vector>

namespace csapex
{
class CSAPEX_UTILS_EXPORT Failure
{
public:
    Failure();
    Failure(const char* msg);
    Failure(const std::string& msg);
    virtual ~Failure();

    virtual std::string type() const;
    virtual std::string what() const;
    virtual std::ostream& reason(std::ostream& ss) const;
    virtual std::ostream& stackTrace(std::ostream& ss, std::size_t depth = 100) const;

    virtual Failure* clone() const;

    void printStackTrace() const;

public:
    static constexpr size_t max_depth = 100;

    std::string msg;

private:
    size_t stack_depth_;
    std::vector<std::string> stack_strings_;
};

class CSAPEX_UTILS_EXPORT HardAssertionFailure : public Failure
{
public:
    HardAssertionFailure();
    HardAssertionFailure(const char* msg, const char* code, const char* file, int line, const char* signature);
    HardAssertionFailure(const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& signature);
    ~HardAssertionFailure() override;

    std::ostream& reason(std::ostream& ss) const override;
    std::string what() const override;
    std::string type() const override;

    Failure* clone() const override;

public:
    std::string code;
    std::string file;
    int line;
    std::string signature;

    std::string thread;
};

}  // namespace csapex

#endif  // EXECPTIONS_H
