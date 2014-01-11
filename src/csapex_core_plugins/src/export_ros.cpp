/// HEADER
#include "export_ros.h"

/// COMPONENT
#include <csapex_core_plugins/ros_message_conversion.h>
#include <csapex_core_plugins/ros_handler.h>

/// PROJECT

#include <csapex/model/connector_in.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/model/message.h>

/// SYSTEM
#include <QPushButton>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ExportRos, csapex::Node)

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
        connector_ = addInput<connection_types::AnyMessage>("Anything");

        setSynchronizedInputs(true);

        topic_ = new QLineEdit;
        QPushButton* send = new QPushButton("set");

        QHBoxLayout* sub = new QHBoxLayout;

        sub->addWidget(topic_);
        sub->addWidget(send);

        layout->addLayout(sub);

        connect(send, SIGNAL(clicked()), this, SLOT(updateTopic()));

    }
}

void ExportRos::allConnectorsArrived()
{
    if(state.topic.empty()) {
        return;
    }

    ConnectionType::Ptr msg = connector_->getMessage<ConnectionType>();
//    connection_types::PossibleRosMessage::Ptr prm = boost::dynamic_pointer_cast<connection_types::PossibleRosMessage>(msg);
//    if(prm && prm->isRosMessage()) {
////        throw std::runtime_error("ros message support not implemented");
//        if(create_pub) {
//            ros::AdvertiseOptions ops;

//            prm->info(ops.md5sum, ops.datatype, ops.message_definition, ops.has_header);

//            ops.latch = true;

//            pub = ROSHandler::instance().nh()->advertise(ops);
//            create_pub = false;
//            has_pub = true;
//        }

//        prm->publish(pub);

//    } else {
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
//    }
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
