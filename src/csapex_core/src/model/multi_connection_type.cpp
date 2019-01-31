/// HEADER
#include <csapex/model/multi_connection_type.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>
#include <csapex/utility/register_msg.h>

/// SYSTEM
#include <sstream>
#include <yaml-cpp/yaml.h>

CSAPEX_REGISTER_MESSAGE(csapex::MultiTokenData)

using namespace csapex;
namespace
{
std::string toString(const std::vector<TokenData::Ptr>& types)
{
    std::stringstream ss;
    int i = 0;
    for (std::vector<TokenData::Ptr>::const_iterator it = types.begin(); it != types.end(); ++it) {
        if (i++ > 0)
            ss << ", ";
        ss << (*it)->typeName();
    }
    return ss.str();
}
}  // namespace

MultiTokenData::MultiTokenData()
{
}

MultiTokenData::MultiTokenData(const std::vector<TokenData::Ptr>& types) : TokenData("MultiTokenData", "one of {" + toString(types) + "}"), types_(types)
{
}

bool MultiTokenData::canConnectTo(const TokenData* other_side) const
{
    for (std::vector<TokenData::Ptr>::const_iterator it = types_.begin(); it != types_.end(); ++it) {
        if ((*it)->canConnectTo(other_side)) {
            return true;
        }
    }

    return false;
}

bool MultiTokenData::acceptsConnectionFrom(const TokenData* other_side) const
{
    for (std::vector<TokenData::Ptr>::const_iterator it = types_.begin(); it != types_.end(); ++it) {
        if ((*it)->acceptsConnectionFrom(other_side)) {
            return true;
        }
    }

    return false;
}

void MultiTokenData::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    TokenData::serialize(data, version);
    data << types_;
}
void MultiTokenData::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    TokenData::deserialize(data, version);
    data >> types_;
}

std::vector<TokenData::Ptr> MultiTokenData::getTypes() const {
    return types_;
}



/// YAML
namespace YAML {

template <>
struct as_if<csapex::MultiTokenData, void> {
  explicit as_if(const Node& node_) : node(node_) {}
  const Node& node;

  const csapex::MultiTokenData operator()() const {
    if (!node.m_pNode)
      throw TypedBadConversion<csapex::MultiTokenData>();

    csapex::MultiTokenData t(std::vector<TokenData::Ptr>{});
    if (convert<csapex::MultiTokenData>::decode(node, t))
      return t;
    throw TypedBadConversion<csapex::MultiTokenData>();
  }
};

Node convert<csapex::MultiTokenData>::encode(const csapex::MultiTokenData& rhs)
{
    YAML::Node node;
    node["types"] = rhs.getTypes();
    return node;
}

bool convert<csapex::MultiTokenData>::decode(const Node& node, csapex::MultiTokenData& rhs)
{
    rhs = csapex::MultiTokenData(node["types"].as<std::vector<TokenData::Ptr>>());
    return true;
}
}
