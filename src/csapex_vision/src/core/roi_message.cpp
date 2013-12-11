/// HEADER
#include <csapex_vision/roi_message.h>

using namespace csapex;
using namespace connection_types;


RoiMessage::RoiMessage()
    : MessageTemplate<cv::Rect, RoiMessage> ("cv::Rect")
{}

void RoiMessage::writeYaml(YAML::Emitter &yaml)
{
    yaml << YAML::Flow << YAML::Key << "value" << YAML::Value;
    yaml << YAML::BeginSeq;
    yaml << value.x << value.y << value.width << value.height;
    yaml << YAML::EndSeq;
}

void RoiMessage::readYaml(const YAML::Node &node)
{
    if(node.FindValue("value")) {
        const YAML::Node& n = node["value"];
        assert(n.Type() == YAML::NodeType::Sequence);
        n[0] >> value.x;
        n[1] >> value.y;
        n[2] >> value.width;
        n[3] >> value.height;
    }
}
