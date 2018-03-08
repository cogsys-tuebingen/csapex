/// HEADER
#include <csapex/msg/token_traits.h>

/// COMPONENT
#include <csapex/model/token.h>

using namespace csapex;
using namespace connection_types;


TokenPtr csapex::connection_types::makeToken(const TokenDataConstPtr& data)
{
    return std::make_shared<Token>(data);
}
