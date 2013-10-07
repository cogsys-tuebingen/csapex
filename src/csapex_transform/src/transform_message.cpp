/// HEADER
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <tf/transform_datatypes.h>

using namespace csapex;
using namespace connection_types;

TransformMessage::TransformMessage()
    : MessageTemplate<tf::Transform, TransformMessage> ("Transform")
{}

void TransformMessage::writeYaml(YAML::Emitter& yaml) {
    const tf::Quaternion& q = value.getRotation();
    const tf::Vector3& t = value.getOrigin();

    yaml << YAML::Key << "orientation" << YAML::Value << YAML::Flow << YAML::BeginSeq << q.x() << q.y() << q.z() << q.w() << YAML::EndSeq;
    yaml << YAML::Key << "translation" << YAML::Value << YAML::Flow << YAML::BeginSeq << t.x() << t.y() << t.z() << YAML::EndSeq;;
}

void TransformMessage::readYaml(YAML::Node& node) {
    double qx,qy,qz,qw,x,y,z;
    if(node.FindValue("orientation")) {
        const YAML::Node& doc = node["orientation"];
        assert(doc.Type() == YAML::NodeType::Sequence);
        doc[0] >> qx;
        doc[1] >> qy;
        doc[2] >> qz;
        doc[3] >> qw;
    }
    if(node.FindValue("translation")) {
        const YAML::Node& doc = node["translation"];
        assert(doc.Type() == YAML::NodeType::Sequence);
        doc[0] >> x;
        doc[1] >> y;
        doc[2] >> z;
    }

    value = tf::Transform(tf::Quaternion(qx,qy,qz,qw), tf::Vector3(x, y, z));
}
