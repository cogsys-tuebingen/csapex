/// HEADER
#include <csapex/utility/stream_relay.h>

using namespace csapex;

StreamRelay::StreamRelay(std::ostream &stream, const std::string &prefix)
    : s_(stream), has_prefix_(true), prefix_(prefix), continued_(new StreamRelay(s_))
{

}
StreamRelay::StreamRelay(std::ostream &stream)
    : s_(stream), has_prefix_(false), continued_(this)
{

}

StreamRelay::~StreamRelay()
{
    if(continued_ != this) {
        delete continued_;
    }
}

void StreamRelay::setPrefix(const std::string &prefix)
{
    prefix_ = prefix;
}

void StreamRelay::writePrefix()
{
    s_ << "[" << prefix_ << "] ";
}
