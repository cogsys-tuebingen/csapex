#ifndef STREAM_RELAY_H
#define STREAM_RELAY_H

/// SYSTEM
#include <string>
#include <iostream>

namespace csapex
{
class StreamRelay : public std::ostream
{
public:
    StreamRelay(std::ostream& stream, const std::string& prefix);
    StreamRelay(std::ostream& stream);
    ~StreamRelay();

    void setPrefix(const std::string& prefix);

    template <class Type>
    StreamRelay& operator << (const Type &x) {
        if(has_prefix_) {
            writePrefix();
        }
        s_ << x;
        return *continued_;
    }


    typedef std::ostream& (*ostream_manipulator)(std::ostream&);

    StreamRelay& operator<<(std::ostream& (*pf)(std::ostream&))
    {
        s_ << pf;
        return *continued_;
    }


private:
    void writePrefix();

private:
    std::ostream& s_;

    bool has_prefix_;
    std::string prefix_;

    StreamRelay* continued_;
};
}

#endif // STREAM_RELAY_H
