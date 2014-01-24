/// HEADER
#include "time_offset.h"

/// COMPONENT
#include <csapex_transform/time_stamp_message.h>

/// PROJECT

#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TimeOffset, csapex::Node)

using namespace csapex;

TimeOffset::TimeOffset()
{
    addTag(Tag::get("Time"));
}


void TimeOffset::allConnectorsArrived()
{
    connection_types::TimeStampMessage::Ptr in = input_->getMessage<connection_types::TimeStampMessage>();
    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);

    if(in->value.toNSec() != 0) {
        std::cerr << in->value.toNSec() << " + " << state.offset_ms_ << " * " << 1e6 << " = " << (in->value.toNSec() + state.offset_ms_ * 1e6) << std::endl,
                time->value = time->value.fromNSec((in->value.toNSec() + state.offset_ms_ * 1000000));
        setError(false);
    } else {
        setError(true, "Time is 0, using current time as base", EL_WARNING);
        ros::Time now = ros::Time::now();
        time->value = now - ros::Duration(0, state.offset_ms_ * 1000000);
    }
    output_->publish(time);
}

void TimeOffset::fill(QBoxLayout* layout)
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::TimeStampMessage>("Time");
    output_ = addOutput<connection_types::TimeStampMessage>("Time");

    offset_ = QtHelper::makeDoubleSlider(layout, "offset (ms)", 0.0, -5000.0, 5000.0, 0.5);
    QObject::connect(offset_, SIGNAL(valueChanged(double)), this, SLOT(update()));

    update();
}


void TimeOffset::update()
{
    state.offset_ms_ = offset_->doubleValue();
}

Memento::Ptr TimeOffset::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void TimeOffset::setState(Memento::Ptr memento)
{
    boost::shared_ptr<TimeOffset::State> m = boost::dynamic_pointer_cast<TimeOffset::State> (memento);
    assert(m.get());

    state = *m;

    offset_->setDoubleValue(state.offset_ms_);
}

void TimeOffset::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "offset" << YAML::Value << offset_ms_;
}
void TimeOffset::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("offset")) {
        node["offset"] >> offset_ms_;
    }
}
