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
    input_(NULL), channel_count_(0)
{
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

    if(channel_count_ != channels.size()) {
        channel_count_ = channels.size();
        updateOutputs();
    }

    for(int i = 0 ; i < channel_count_ ; i++) {
        CvMatMessage::Ptr channel_out(new CvMatMessage);
        channel_out->value = channels[i];
        box_->getOutput(i)->publish(channel_out);
    }
}

void Splitter::updateOutputs()
{
    command::Meta::Ptr cmd(new command::Meta);

    for(int i = 0 ; i < box_->countOutputs() ; i++) {
        ConnectorOut *ptr = box_->getOutput(i);
        if(ptr->isConnected())
            cmd->add(ptr->removeAllConnectionsCmd());
        box_->removeOutput(box_->getOutput(i));
    }

    BoxManager::instance().execute(cmd);

    for(int i = 0 ; i < channel_count_ ; i++) {
        ConnectorOut *out = new ConnectorOut(box_, i);
        box_->addOutput(out);
    }
}
