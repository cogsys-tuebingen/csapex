/// HEADER
#include "filter.h"

/// COMPONENT
#include "messages_default.hpp"

/// PROJECT
#include <designer/connector_in.h>
#include <designer/connector_out.h>
#include <designer/box.h>

/// SYSTEM
#include <QLabel>

using namespace vision_evaluator;


Filter::Filter()
    : input_img_(NULL), input_mask_(NULL), output_img_(NULL), output_mask_(NULL), has_img(false), has_mask(false)
{
    setName("unnamed filter");
}

Filter::~Filter()
{
}

void Filter::fill(QBoxLayout* parent)
{
    if(input_img_ == NULL) {

        input_img_ = new ConnectorIn(box_, 0);
        box_->addInput(input_img_);
        input_mask_ = new ConnectorIn(box_, 1);
        box_->addInput(input_mask_);

        output_img_ = new ConnectorOut(box_, 0);
        box_->addOutput(output_img_);
        output_mask_ = new ConnectorOut(box_, 1);
        box_->addOutput(output_mask_);

        connect(input_img_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*)));
        connect(input_mask_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*)));

        insert(parent);

        makeThread();
        this->moveToThread(private_thread_);
        private_thread_->start();
    }
}

void Filter::messageArrived(ConnectorIn* source)
{
    if(!isEnabled()) {
        return;
    }

    if(source == input_img_) {
        has_img = true;
    } else if(source == input_mask_) {
        has_mask = true;
    }

    if(!input_mask_->isConnected()) {
        has_mask = true;
    }


    if(!has_mask || !has_img) {
        return;
    }

    has_img = false;
    has_mask = false;

    CvMatMessage::Ptr img_msg = boost::dynamic_pointer_cast<CvMatMessage> (input_img_->getMessage());
    CvMatMessage::Ptr mask_msg = boost::dynamic_pointer_cast<CvMatMessage> (input_mask_->getMessage());

    if(img_msg.get() && !img_msg->value.empty()) {
        if(!mask_msg.get()){
            mask_msg.reset(new CvMatMessage);
        }
        filter(img_msg->value, mask_msg->value);

        output_img_->publish(img_msg);
        output_mask_->publish(mask_msg);
    }
}
