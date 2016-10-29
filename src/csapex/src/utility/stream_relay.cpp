/// HEADER
#include <csapex/utility/stream_relay.h>

/// SYSTEM
#include <sstream>

using namespace csapex;

StreamRelay::StreamRelay(std::ostream &stream, const std::string &prefix)
    : s_(stream), is_enabled_(true), has_prefix_(true), prefix_(prefix), history_(new std::stringstream),  continued_(new StreamRelay(s_, history_))
{

}
StreamRelay::StreamRelay(std::ostream &stream, std::shared_ptr<std::stringstream> history)
    : s_(stream), is_enabled_(true), has_prefix_(false), history_(history)
{

}

StreamRelay::~StreamRelay()
{
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


StreamRelay& StreamRelay::operator<<(std::ostream& (*pf)(std::ostream&))
{
    if(is_enabled_) {
        *history_ << pf;
        s_ << pf;
    }

    return *continued_;
}

void StreamRelay::setEnabled(bool enable)
{
    if(is_enabled_ != enable) {
        is_enabled_ = enable;
        if(continued_) {
            continued_->setEnabled(enable);
        }
    }
}

bool StreamRelay::isEnabled() const
{
    return is_enabled_;
}
