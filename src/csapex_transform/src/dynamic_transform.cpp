/// HEADER
#include "dynamic_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_transform/time_stamp_message.h>
#include "listener.h"

/// PROJECT

#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex_core_plugins/string_message.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <tf/transform_datatypes.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::DynamicTransform, csapex::Node)

using namespace csapex;

DynamicTransform::DynamicTransform()
    : from_box_(NULL), to_box_(NULL)
{
    addTag(Tag::get("Transform"));
}

void DynamicTransform::process()
{
    setError(false);
    bool update = false;
    if(frame_in_from_->isConnected() && frame_in_from_->hasMessage()) {
        std::string from = frame_in_from_->getMessage<connection_types::StringMessage>()->value;

        if(state.from_ != from) {
            state.from_ = from;
            update = true;
        }
        if(from_box_) {
            from_box_->setEnabled(false);
        }
    } else {
        if(from_box_) {
            from_box_->setEnabled(true);
        }
    }

    if(frame_in_to_->isConnected() && frame_in_to_->hasMessage()) {
        std::string to = frame_in_to_->getMessage<connection_types::StringMessage>()->value;

        if(state.to_ != to) {
            state.to_ = to;
            update = true;
        }
        if(to_box_){
            to_box_->setEnabled(false);
        }
    } else {
        if(to_box_){
            to_box_->setEnabled(true);
        }
    }

    if(update) {
        updateFrames();
    }


    connection_types::TimeStampMessage::Ptr time_msg = time_in_->getMessage<connection_types::TimeStampMessage>();
    publishTransform(time_msg->value);
}

void DynamicTransform::tick()
{
    if(time_in_->isConnected()) {
        return;
    }

    publishTransform(ros::Time(0));
}

void DynamicTransform::publishTransform(const ros::Time& time)
{
    tf::StampedTransform t;

    try {
        LockedListener l = Listener::getLocked();

        if(l.l) {
            l.l->tfl->lookupTransform(state.to_, state.from_, time, t);
            setError(false);
        } else {
            return;
        }
    } catch(const tf2::TransformException& e) {
        setError(true, e.what(), EL_WARNING);
        return;
    }


    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = t;
    output_->publish(msg);

    connection_types::StringMessage::Ptr frame(new connection_types::StringMessage);
    frame->value = state.to_;
    output_frame_->publish(frame);
}

void DynamicTransform::setup()
{
    setSynchronizedInputs(true);

    time_in_ = addInput<connection_types::TimeStampMessage>("Time", true);
    frame_in_from_ = addInput<connection_types::StringMessage>("Origin Frame", true, true);
    frame_in_to_ = addInput<connection_types::StringMessage>("Target Frame", true, true);

    output_ = addOutput<connection_types::TransformMessage>("Transform");
    output_frame_ = addOutput<connection_types::StringMessage>("Target Frame");
}

void DynamicTransform::fill(QBoxLayout* layout)
{
    from_box_ = new QComboBox;
    from_box_->setEditable(true);
    layout->addWidget(from_box_);

    to_box_ = new QComboBox;
    to_box_->setEditable(true);
    layout->addWidget(to_box_);

    refresh_ = new QPushButton("refresh");
    QObject::connect(refresh_, SIGNAL(clicked()), this, SLOT(updateFrames()));
    layout->addWidget(refresh_);

    reset_tf_ = new QPushButton("reset tf");
    QObject::connect(reset_tf_, SIGNAL(clicked()), this, SLOT(resetTf()));
    layout->addWidget(reset_tf_);

    QObject::connect(from_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(update()));
    QObject::connect(from_box_, SIGNAL(editTextChanged(QString)), this, SLOT(update()));
    QObject::connect(to_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(update()));
    QObject::connect(to_box_, SIGNAL(editTextChanged(QString)), this, SLOT(update()));

    QObject::connect(this, SIGNAL(started()), this, SLOT(updateFrames()));

    ROSHandler::instance().registerConnectionCallback(boost::bind(&DynamicTransform::updateFrames, this));

    updateFrames();
}

void DynamicTransform::resetTf()
{
    LockedListener l = Listener::getLocked();
    if(l.l) {
        l.l->reset();
    }
}

void DynamicTransform::updateFrames()
{
    std::vector<std::string> frames;

    LockedListener l = Listener::getLocked();
    if(l.l) {
        l.l->tfl->getFrameStrings(frames);
    } else {
        return;
    }

    if(from_box_) {
        blockSignals(true);

        from_box_->clear();
        to_box_->clear();
        int i = 0;
        bool has_from = false;
        bool has_to = false;

        BOOST_FOREACH(const std::string& frame, frames) {
            from_box_->addItem(frame.c_str());
            to_box_->addItem(frame.c_str());

            if(frame == state.from_) {
                from_box_->setCurrentIndex(i);
                has_from = true;
            }
            if(frame == state.to_) {
                to_box_->setCurrentIndex(i);
                has_to = true;
            }

            ++i;
        }

        if(!has_from) {
            from_box_->addItem(state.from_.c_str());
            from_box_->setCurrentIndex(i);
        }

        if(!has_to) {
            to_box_->addItem(state.to_.c_str());
            to_box_->setCurrentIndex(i);
        }

        blockSignals(false);
    }
}

void DynamicTransform::update()
{
    if(signalsBlocked()) {
        return;
    }

    if(from_box_->currentText().length() == 0 || to_box_->currentText().length() == 0) {
        return;
    }

    if(from_box_->model()->rowCount() == 0 || to_box_->model()->rowCount() == 0) {
        return;
    }

    state.from_ = from_box_->currentText().toStdString();
    state.to_ = to_box_->currentText().toStdString();
}

Memento::Ptr DynamicTransform::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void DynamicTransform::setState(Memento::Ptr memento)
{
    boost::shared_ptr<DynamicTransform::State> m = boost::dynamic_pointer_cast<DynamicTransform::State> (memento);
    assert(m.get());

    state = *m;

    updateFrames();
}

void DynamicTransform::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "from" << YAML::Value << from_;
    out << YAML::Key << "to" << YAML::Value << to_;
}
void DynamicTransform::State::readYaml(const YAML::Node& node) {
    if(exists(node, "from")) {
        node["from"] >> from_;
    }
    if(exists(node, "to")) {
        node["to"] >> to_;
    }
}
