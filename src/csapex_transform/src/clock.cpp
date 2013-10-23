/// HEADER
#include "clock.h"

/// COMPONENT
#include <csapex_transform/time_stamp_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex_core_plugins/ros_handler.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/date_time.hpp>

CSAPEX_REGISTER_CLASS(csapex::Clock, csapex::BoxedObject)

using namespace csapex;

Clock::Clock()
{
    addTag(Tag::get("Time"));
}


void Clock::tick()
{
    ROSHandler::instance().waitForConnection();

    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    if(state.use_ros_time) {
        time->value = ros::Time::now();
    } else {
        time->value = ros::Time(0);
    }

    time_label_->setText(boost::posix_time::to_simple_string(time->value.toBoost()).c_str());

    output_->publish(time);
}

void Clock::fill(QBoxLayout* layout)
{
    setSynchronizedInputs(true);

    output_ = addOutput<connection_types::TimeStampMessage>("Time");

    time_type_ = new QPushButton;
    time_type_->setCheckable(true);
    QObject::connect(time_type_, SIGNAL(clicked()), this, SLOT(update()));
    layout->addWidget(time_type_);

    time_label_ = new QLabel;
    layout->addWidget(time_label_);

    update();
}


void Clock::update()
{
    state.use_ros_time = time_type_->isChecked();

    if(state.use_ros_time) {
        time_type_->setText("ros::Time::now()");
    } else {
        time_type_->setText("ros::Time(0)");
    }
}

Memento::Ptr Clock::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void Clock::setState(Memento::Ptr memento)
{
    boost::shared_ptr<Clock::State> m = boost::dynamic_pointer_cast<Clock::State> (memento);
    assert(m.get());

    state = *m;

    update();
}

void Clock::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "use_ros_time" << YAML::Value << use_ros_time;
}
void Clock::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("use_ros_time")) {
        node["use_ros_time"] >> use_ros_time;
    }
}
