/// HEADER
#include "export_ros.h"

/// COMPONENT
#include <csapex_core_plugins/ros_message_conversion.h>
#include <csapex_core_plugins/ros_handler.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/model/message.h>

/// SYSTEM
#include <QPushButton>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::ExportRos, csapex::BoxedObject)

using namespace csapex;

ExportRos::ExportRos()
    : connector_(NULL), has_pub(false), create_pub(false)
{
    addTag(Tag::get("RosIO"));
    addTag(Tag::get("General"));
    addTag(Tag::get("Output"));
    setIcon(QIcon(":/terminal.png"));
}

void ExportRos::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = new ConnectorIn(box_, 0);
        connector_->setLabel("Anything");
        connector_->setType(connection_types::AnyMessage::make());

        topic_ = new QLineEdit;
        QPushButton* send = new QPushButton("set");

        QHBoxLayout* sub = new QHBoxLayout;

        sub->addWidget(topic_);
        sub->addWidget(send);

        layout->addLayout(sub);

        connect(send, SIGNAL(clicked()), this, SLOT(updateTopic()));

        box_->addInput(connector_);
    }
}

void ExportRos::messageArrived(ConnectorIn *source)
{
    if(state.topic.empty()) {
        return;
    }

    ConnectionType::Ptr msg = source->getMessage();

    if(create_pub) {
        pub = RosMessageConversion::instance().advertise(msg->toType(), state.topic, 1, true);
        create_pub = false;
        has_pub = true;

        connector_->setLabel(pub.getTopic());
        connector_->setType(msg);
    }

    if(!has_pub) {
        setError(true, "Publisher is not valid", EL_WARNING);
        return;
    }

    RosMessageConversion::instance().publish(pub, msg);
}

void ExportRos::updateTopic()
{
    state.topic = topic_->text().toStdString();
    std::cout << "trying to publish @" << state.topic << std::endl;
    create_pub = true;
}

void ExportRos::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "topic" << YAML::Value << topic;
}

void ExportRos::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("topic"))
        node["topic"] >> topic;
}

Memento::Ptr ExportRos::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ExportRos::setState(Memento::Ptr memento)
{
    boost::shared_ptr<ExportRos::State> m = boost::dynamic_pointer_cast<ExportRos::State> (memento);
    assert(m.get());

    state = *m;

    create_pub = true;

    topic_->setText(state.topic.c_str());
}
