#include "filter_splitter.h"

/// PROJECT
#include <designer/box.h>
#include <designer/command_meta.h>
#include <vision_evaluator/box_manager.h>
#include <evaluator/messages_default.hpp>
#include <designer/connector_in.h>
#include <designer/connector_out.h>


/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::Splitter, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

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

        assert(QObject::connect(input_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*))));
    }
}

void Splitter::messageArrived(ConnectorIn *source)
{
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());
    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    if(state_.channel_count_ != channels.size()) {
        state_.channel_count_ = channels.size();
        updateOutputs();
    }

    for(int i = 0 ; i < state_.channel_count_ ; i++) {
        CvMatMessage::Ptr channel_out(new CvMatMessage);
        channel_out->value = channels[i];
        box_->getOutput(i)->publish(channel_out);
    }
}

void Splitter::updateOutputs()
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

    updateOutputs();
}
