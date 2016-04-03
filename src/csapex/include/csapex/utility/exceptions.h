#ifndef EXECPTIONS_H
#define EXECPTIONS_H

#include <exception>
#include <string>
#include <vector>

namespace csapex
{

class Failure
{
public:
    Failure();
    Failure(const char* msg);
    Failure(const std::string& msg);
    virtual ~Failure();

    virtual std::string type() const;
    virtual std::string what() const;
    virtual std::ostream& reason(std::ostream& ss) const;
    virtual std::ostream& stackTrace(std::ostream& ss, std::size_t depth = max_depth) const;

    virtual Failure* clone() const;

    void printStackTrace() const;

public:
    static constexpr size_t max_depth = 100;

    std::string msg;

private:
    size_t stack_depth_;
    std::vector<std::string> stack_strings_;
};

class HardAssertionFailure : public Failure
{
public:
    HardAssertionFailure();
    HardAssertionFailure(const char* msg, const char* code, const char* file, int line);
    ~HardAssertionFailure();

    virtual std::ostream& reason(std::ostream& ss) const override;
    virtual std::string what() const override;
    virtual std::string type() const override;

    virtual Failure* clone() const override;

public:
    std::string code;
    std::string file;
    int line;

    std::string thread;
};

}

#endif // EXECPTIONS_H
