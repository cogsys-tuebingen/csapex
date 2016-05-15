/// HEADER
#include <csapex/model/token.h>

using namespace csapex;

/***
 * MESSAGE FLAGS
 */
Token::Flags::Flags()
    : data(0)
{

}


Token::Token(const TokenDataConstPtr &token)
    : token_(token), active_(false), seq_no_(-1)
{

}

void Token::setActive(bool active)
{
    active_ = active;
}

bool Token::isActive() const
{
    return active_;
}


TokenDataConstPtr Token::getTokenData() const
{
    return token_;
}


int Token::getSequenceNumber() const
{
    return seq_no_;
}

void Token::setSequenceNumber(int seq_no) const
{
    seq_no_ = seq_no;
}
