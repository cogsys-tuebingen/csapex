/// HEADER
#include "time_offset.h"

/// COMPONENT
#include <csapex_transform/time_stamp_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::TimeOffset, csapex::BoxedObject)

using namespace csapex;

TimeOffset::TimeOffset()
{
    addTag(Tag::get("Time"));
}


void TimeOffset::allConnectorsArrived()
{
    connection_types::TimeStampMessage::Ptr in = input_->getMessage<connection_types::TimeStampMessage>();
    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    time->value = time->value.fromNSec((in->value.toNSec() + state.offset_ms_ * 1e6));

//    std::cout << "before: \t" << in->value.toNSec() << "\nafter: \t" << time->value.toNSec() << "\noffset: \t" << 1e6 * state.offset_ms_ << std::endl;
    output_->publish(time);
}

void TimeOffset::fill(QBoxLayout* layout)
{
    box_->setSynchronizedInputs(true);

    output_ = new ConnectorOut(box_, 0);
    output_->setType(connection_types::TimeStampMessage::make());
    box_->addOutput(output_);

    input_ = new ConnectorIn(box_, 0);
    input_->setType(connection_types::TimeStampMessage::make());
    box_->addInput(input_);

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
