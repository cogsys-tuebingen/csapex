#ifndef ENCODING_H
#define ENCODING_H

/// SYSTEM
#include <string>
#include <vector>
#include <boost/assign/list_of.hpp>

namespace csapex {

struct Channel {
    friend bool operator == (const Channel& lhs, const Channel& rhs) {
        return lhs.name == rhs.name;
    }

    friend bool operator != (const Channel& lhs, const Channel& rhs) {
        return ! operator == (lhs, rhs);
    }

    Channel(const std::string& name, int min, int max)
        : name(name), min(min), max(max)
    {}

    std::string name;
    int min;
    int max;
};

class Encoding : public std::vector<Channel>
{
    typedef std::vector<Channel> Parent;

    friend bool operator != (const Encoding& lhs, const Encoding& rhs) {
        return ! operator == (lhs, rhs);
    }

    friend bool operator == (const Encoding& lhs, const Encoding& rhs) {
        if(lhs.size() != rhs.size()) {
            return false;
        }

        for(unsigned i = 0; i < lhs.size(); ++i) {
            if(lhs[i] != rhs[i]) {
                return false;
            }
        }

        return true;
    }

public:
    template <typename Iterator>
    Encoding(Iterator begin, Iterator end)
        : Parent(begin, end)
    {}

    Encoding();

    std::string toString() const;
};

namespace enc {
static const Encoding mono = boost::assign::list_of
        (Channel("gray",0,255));

static const Encoding bgr = boost::assign::list_of
        (Channel("b",0,255))
        (Channel("g",0,255))
        (Channel("r",0,255));

static const Encoding unknown = bgr;

static const Encoding rgb = boost::assign::list_of
        (Channel("r",0,255))
        (Channel("g",0,255))
        (Channel("b",0,255));

static const Encoding hsv = boost::assign::list_of
        (Channel("h",0,255))
        (Channel("s",0,255))
        (Channel("v",0,255));

static const Encoding hsl = boost::assign::list_of
        (Channel("h",0,255))
        (Channel("s",0,255))
        (Channel("l",0,255));

static const Encoding yuv = boost::assign::list_of
        (Channel("y",0,255))
        (Channel("u",0,255))
        (Channel("v",0,255));

static const Encoding depth = boost::assign::list_of
        (Channel("d",0,255));
}

}

#endif // ENCODING_H
