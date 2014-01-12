/// HEADER
#include <csapex_vision/roi_message.h>

using namespace csapex;
using namespace connection_types;


RoiMessage::RoiMessage()
    : MessageTemplate<Roi, RoiMessage> ("Roi")
{}

void RoiMessage::writeYaml(YAML::Emitter &yaml)
{
    yaml << YAML::Flow << YAML::Key << "value" << YAML::Value;
    yaml << YAML::BeginSeq;
    yaml << value.x() << value.y() << value.w() << value.h();
    yaml << YAML::EndSeq;
}

void RoiMessage::readYaml(const YAML::Node &node)
{
    if(node.FindValue("value")) {
        const YAML::Node& n = node["value"];
        assert(n.Type() == YAML::NodeType::Sequence);

        int x,y,w,h;
        n[0] >> x;
        n[1] >> y;
        n[2] >> w;
        n[3] >> h;

        value = Roi(x,y,w,h);
    }
}
