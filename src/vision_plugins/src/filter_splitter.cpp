#include "filter_splitter.h"

/// PROJECT
#include <csapex/box.h>
#include <csapex/command_meta.h>
#include <csapex_vision/messages_default.hpp>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>


/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::Splitter, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

Splitter::Splitter() :
    input_(NULL)
{
    state_.channel_count_ = 0;
}

void Splitter::fill(QBoxLayout *layout)
{
    if(input_ == NULL) {
        /// add input
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);
    }
}

void Splitter::messageArrived(ConnectorIn *source)
{
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());
    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    if(state_.channel_count_ != channels.size()) {
        state_.channel_count_ = channels.size();
        Q_EMIT modelChanged();
    }

    for(int i = 0 ; i < box_->countOutputs() ; i++) {
        CvMatMessage::Ptr channel_out(new CvMatMessage);
        channel_out->value = channels[i];
        box_->getOutput(i)->publish(channel_out);
    }
}

void Splitter::updateDynamicGui(QBoxLayout *layout)
{
    int n = box_->countOutputs();

    if(state_.channel_count_ == n) {
        return;
    }

    if(state_.channel_count_ > n) {
        for(int i = n ; i < state_.channel_count_ ; ++i) {
            ConnectorOut *out = new ConnectorOut(box_, i);
            box_->addOutput(out);
        }
    } else {
        for(int i = n-1 ; i >= state_.channel_count_ ; --i) {
            box_->removeOutput(box_->getOutput(i));
        }
    }
}

/// MEMENTO ------------------------------------------------------------------------------------
Memento::Ptr Splitter::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void Splitter::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;

    box_->eventModelChanged();
    Q_EMIT modelChanged();
}

/// MEMENTO
void Splitter::State::readYaml(const YAML::Node &node)
{
    node["channel_count"] >> channel_count_;
}


void Splitter::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "channel_count" << YAML::Value << channel_count_;
}
