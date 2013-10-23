/// HEADER
#include "static_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <tf/transform_datatypes.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::StaticTransform, csapex::BoxedObject)

using namespace csapex;

StaticTransform::StaticTransform()
{
    addTag(Tag::get("Transform"));
}

void StaticTransform::tick()
{
    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = tf::Transform(tf::createQuaternionFromRPY(state.roll, state.pitch, state.yaw), tf::Vector3(state.x, state.y, state.z));
    output_->publish(msg);
}


void StaticTransform::fill(QBoxLayout* layout)
{
    setSynchronizedInputs(true);

    output_ = addOutput<connection_types::TransformMessage>("Transformation");

    double p = 3.1415;
    roll_ = QtHelper::makeDoubleSlider(layout,  "roll",  0.0, -p, p, 0.001);
    pitch_ = QtHelper::makeDoubleSlider(layout, "pitch", 0.0, -p, p, 0.001);
    yaw_ = QtHelper::makeDoubleSlider(layout,   "yaw",   0.0, -p, p, 0.001);

    double d = 5.0;
    x_ = QtHelper::makeDoubleSlider(layout, "dx", 0.0, -d, d, 0.01);
    y_ = QtHelper::makeDoubleSlider(layout, "dy", 0.0, -d, d, 0.01);
    z_ = QtHelper::makeDoubleSlider(layout, "dz", 0.0, -d, d, 0.01);

    QObject::connect(roll_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(pitch_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(yaw_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(x_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(y_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(z_, SIGNAL(valueChanged(double)), this, SLOT(update()));

}

void StaticTransform::update()
{
    if(!signalsBlocked()) {
        state.roll = roll_->doubleValue();
        state.pitch = pitch_->doubleValue();
        state.yaw = yaw_->doubleValue();

        state.x = x_->doubleValue();
        state.y = y_->doubleValue();
        state.z = z_->doubleValue();
    }
}

Memento::Ptr StaticTransform::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void StaticTransform::setState(Memento::Ptr memento)
{
    boost::shared_ptr<StaticTransform::State> m = boost::dynamic_pointer_cast<StaticTransform::State> (memento);
    assert(m.get());

    state = *m;

    blockSignals(true);
    roll_->setDoubleValue(state.roll);
    pitch_->setDoubleValue(state.pitch);
    yaw_->setDoubleValue(state.yaw);
    x_->setDoubleValue(state.x);
    y_->setDoubleValue(state.y);
    z_->setDoubleValue(state.z);
    blockSignals(false);
}

void StaticTransform::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "orientation" << YAML::Value << YAML::Flow << YAML::BeginSeq << roll << pitch << yaw << YAML::EndSeq;
    out << YAML::Key << "translation" << YAML::Value << YAML::Flow << YAML::BeginSeq << x << y << z << YAML::EndSeq;;
}
void StaticTransform::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("orientation")) {
        const YAML::Node& doc = node["orientation"];
        assert(doc.Type() == YAML::NodeType::Sequence);
        doc[0] >> roll;
        doc[1] >> pitch;
        doc[2] >> yaw;
    }
    if(node.FindValue("translation")) {
        const YAML::Node& doc = node["translation"];
        assert(doc.Type() == YAML::NodeType::Sequence);
        doc[0] >> x;
        doc[1] >> y;
        doc[2] >> z;
    }
}
