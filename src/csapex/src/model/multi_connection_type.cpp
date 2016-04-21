/// HEADER
#include <csapex/model/multi_connection_type.h>

/// SYSTEM
#include <sstream>

using namespace csapex;

namespace {
std::string toString(const std::vector<Token::Ptr>& types) {
    std::stringstream ss;
    int i = 0;
    for(std::vector<Token::Ptr>::const_iterator it = types.begin(); it != types.end(); ++it) {
        if(i++ > 0) ss << ", ";
        ss << (*it)->typeName();
    }
    return ss.str();
}
}

MultiToken::MultiToken(const std::vector<Token::Ptr>& types)
    : Token("one of {" + toString(types) + "}"), types_(types)
{

}

bool MultiToken::canConnectTo(const Token *other_side) const
{
    for(std::vector<Token::Ptr>::const_iterator it = types_.begin(); it != types_.end(); ++it) {
        if((*it)->canConnectTo(other_side)) {
            return true;
        }
    }

    return false;
}

bool MultiToken::acceptsConnectionFrom(const Token *other_side) const
{
    for(std::vector<Token::Ptr>::const_iterator it = types_.begin(); it != types_.end(); ++it) {
        if((*it)->acceptsConnectionFrom(other_side)) {
            return true;
        }
    }

    return false;
}

Token::Ptr MultiToken::clone() const
{
    Ptr new_msg(new MultiToken(types_));
    return new_msg;
}

Token::Ptr MultiToken::toType() const
{
    Ptr new_msg(new MultiToken(types_));
    return new_msg;
}
