/// HEADER
#include <csapex_vision/filter.h>

/// COMPONENT
#include <csapex_vision/messages_default.hpp>

/// PROJECT
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/box.h>

/// SYSTEM
#include <QLabel>

using namespace csapex;
using namespace connection_types;


Filter::Filter()
    : input_img_(NULL), input_mask_(NULL), output_img_(NULL), output_mask_(NULL), has_img(false), has_mask(false), guard(0xDEADBEEFL)
{
    setName("unnamed filter");
    setCategory("Filter");
    setIcon(QIcon(":/filter.png"));
}

Filter::~Filter()
{
}

void Filter::fill(QBoxLayout* parent)
{
    if(input_img_ == NULL) {

        input_img_ = new ConnectorIn(box_, 0);
        box_->addInput(input_img_);
        if(usesMask()) {
            input_mask_ = new ConnectorIn(box_, 1);
            box_->addInput(input_mask_);
        }
        output_img_ = new ConnectorOut(box_, 0);
        box_->addOutput(output_img_);
        if(usesMask()) {
            output_mask_ = new ConnectorOut(box_, 1);
            box_->addOutput(output_mask_);
        }

        insert(parent);
    }
}

void Filter::messageArrived(ConnectorIn* source)
{
    if(source == input_img_) {
        has_img = true;
    } else if(source == input_mask_) {
        has_mask = true;
    }

    if(!usesMask() || !input_mask_->isConnected()) {
        has_mask = true;
    }


    if(!has_mask || !has_img) {
        return;
    }

    has_img = false;
    has_mask = false;

    ConnectionType::Ptr msg = input_img_->getMessage();
    CvMatMessage::Ptr img_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);
    CvMatMessage::Ptr mask_msg;
    if(usesMask()) {
        mask_msg = boost::dynamic_pointer_cast<CvMatMessage> (input_mask_->getMessage());
    }

    if(img_msg.get() && !img_msg->value.empty()) {
        if(!mask_msg.get()) {
            mask_msg.reset(new CvMatMessage);
        }

        filter(img_msg->value, mask_msg->value);

        output_img_->publish(img_msg);
        if(usesMask())
            output_mask_->publish(mask_msg);
    }
}

bool Filter::usesMask()
{
    return true;
}
