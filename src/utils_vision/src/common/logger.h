#ifndef LOGGER_H
#define LOGGER_H

#include <ostream>

class Logger
{
public:
    static Logger INFO;
    static Logger WARN;
    static Logger ERROR;
    static Logger FATAL;

private:
    Logger(std::ostream& out, const std::string& prefix, const std::string& suffix);

    std::ostream& getOut();

public:
    template<typename T>
    std::ostream& operator << (T arg) {
        return getOut() << arg;
    }

    void endl();

private:
    std::ostream& out_;
    std::string prefix_;
    std::string suffix_;
};

#endif // LOGGER_H
