#ifndef ENCODING_H
#define ENCODING_H

/// SYSTEM
#include <string>
#include <vector>
#include <boost/assign/list_of.hpp>

namespace csapex {

struct Channel {
    Channel(const std::string& name, int min, int max)
        : name(name), min(min), max(max)
    {}

    std::string name;
    int min;
    int max;
};

typedef std::vector<Channel> Encoding;

namespace enc {
static const Encoding mono = boost::assign::list_of
        (Channel("gray",0,255));

static const Encoding bgr = boost::assign::list_of
        (Channel("b",0,255))
        (Channel("g",0,255))
        (Channel("r",0,255));

static const Encoding rgb = boost::assign::list_of
        (Channel("r",0,255))
        (Channel("g",0,255))
        (Channel("b",0,255));

static const Encoding hsv = boost::assign::list_of
        (Channel("h",0,128))
        (Channel("s",0,255))
        (Channel("v",0,255));

static const Encoding hsl = boost::assign::list_of
        (Channel("h",0,128))
        (Channel("s",0,255))
        (Channel("l",0,255));

static const Encoding yuv = boost::assign::list_of
        (Channel("y",0,255))
        (Channel("u",0,255))
        (Channel("v",0,255));
}

}

#endif // ENCODING_H
