/// HEADER
#include <csapex/utility/stream_relay.h>

using namespace csapex;

StreamRelay::StreamRelay(std::ostream &stream, const std::string &prefix)
    : s_(stream), has_prefix_(true), prefix_(prefix), history_(new std::stringstream),  continued_(new StreamRelay(s_, history_))
{

}
StreamRelay::StreamRelay(std::ostream &stream, std::stringstream* history)
    : s_(stream), has_prefix_(false), history_(history), continued_(this)
{

}

StreamRelay::~StreamRelay()
{
    if(continued_ != this) {
        delete continued_;
        delete history_;
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

std::stringstream& StreamRelay::history() const
{
    return *history_;
}
