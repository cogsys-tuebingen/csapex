/// HEADER
#include <csapex/model/token.h>

using namespace csapex;

Token::Token(const TokenDataConstPtr &token)
    : token_(token), activity_modifier_(ActivityModifier::NONE), seq_no_(-1)
{

}

void Token::setActivityModifier(ActivityModifier active)
{
    activity_modifier_ = active;
}

bool Token::hasActivityModifier() const
{
    return activity_modifier_ != ActivityModifier::NONE;
}

ActivityModifier Token::getActivityModifier() const
{
    return activity_modifier_;
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

TokenPtr Token::clone() const
{
    TokenPtr token = std::make_shared<Token>(*this);
    token->token_ = token_->clone();
    return token;
}
