#ifndef STREAM_RELAY_H
#define STREAM_RELAY_H

/// SYSTEM
#include <string>
#include <iostream>
#include <sstream>

namespace csapex
{
class StreamRelay : public std::ostream
{
public:
    StreamRelay(std::ostream& stream, const std::string& prefix);
    ~StreamRelay();

    void setPrefix(const std::string& prefix);

    template <class Type>
    StreamRelay& operator << (const Type &x) {
        if(has_prefix_) {
            writePrefix();
        }
        *history_ << x;
        s_ << x;
        return *continued_;
    }


    typedef std::ostream& (*ostream_manipulator)(std::ostream&);

    StreamRelay& operator<<(std::ostream& (*pf)(std::ostream&))
    {
        *history_ << pf;
        s_ << pf;
        return *continued_;
    }

    std::stringstream& history() const;


private:
    StreamRelay(std::ostream& stream, std::stringstream *history);
    void writePrefix();

private:
    std::ostream& s_;

    bool has_prefix_;
    std::string prefix_;

    mutable std::stringstream* history_;
    StreamRelay* continued_;
};
}

#endif // STREAM_RELAY_H
