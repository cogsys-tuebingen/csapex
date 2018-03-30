/// HEADER
#include <csapex/model/io.h>

/// COMPONENT
#include <csapex/model/token.h>

using namespace csapex;
using namespace model;

TokenDataConstPtr csapex::model::getTokenData(const TokenPtr& token)
{
    return token->getTokenData();
}
