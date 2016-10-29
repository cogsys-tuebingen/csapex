#ifndef STREAM_RELAY_H
#define STREAM_RELAY_H

/// PROJECT
#include <csapex/csapex_util_export.h>

/// SYSTEM
#include <string>
#include <iosfwd>
#include <memory>

namespace csapex
{
class CSAPEX_UTILS_EXPORT StreamRelay
{
public:
    StreamRelay(std::ostream& stream, const std::string& prefix);
    ~StreamRelay();

    void setPrefix(const std::string& prefix);

    template <class Type>
    StreamRelay& operator << (const Type &x) {
        if(is_enabled_) {
            if(has_prefix_) {
                writePrefix();
            }
            *history_ << x;
            s_ << x;
        }
        if(continued_) {
            return *continued_;
        } else {
            return *this;
        }
    }

    void setEnabled(bool muted);
    bool isEnabled() const;

    typedef std::ostream& (*ostream_manipulator)(std::ostream&);

    StreamRelay& operator<<(std::ostream& (*pf)(std::ostream&));

    std::stringstream& history() const;


private:
    StreamRelay(std::ostream& stream, std::shared_ptr<std::stringstream> history);
    void writePrefix();

private:
    std::ostream& s_;

    bool is_enabled_;

    bool has_prefix_;
    std::string prefix_;

    mutable std::shared_ptr<std::stringstream> history_;
    std::unique_ptr<StreamRelay> continued_;
};
}

#endif // STREAM_RELAY_H
