/// HEADER
#include <csapex_vision/image_combiner.h>

/// COMPONENT
#include <csapex_vision/messages_default.hpp>

/// PROJECT
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/box.h>

using namespace csapex;
using namespace connection_types;

ImageCombiner::ImageCombiner()
    : input_img_a_(NULL), input_mask_a_(NULL), input_img_b_(NULL), input_mask_b_(NULL), output_img_(NULL),
      has_img_a(false), has_mask_a(false), has_img_b(false), has_mask_b(false)
{
    addTag(Tag::get("Image Combiner"));
    addTag(Tag::get("Vision"));
    setIcon(QIcon(":/combiner.png"));
}

ImageCombiner::~ImageCombiner()
{

}

void ImageCombiner::fill(QBoxLayout* layout)
{
    if(input_img_a_ == NULL) {
        input_img_a_ = new ConnectorIn(box_, 0);
        input_img_a_->setLabel("Image 1");
        box_->addInput(input_img_a_);
        input_mask_a_ = new ConnectorIn(box_, 1);
        input_mask_a_->setLabel("Mask 1");
        box_->addInput(input_mask_a_);
        input_img_b_ = new ConnectorIn(box_, 2);
        input_img_b_->setLabel("Image 2");
        box_->addInput(input_img_b_);
        input_mask_b_ = new ConnectorIn(box_, 3);
        input_mask_b_->setLabel("Mask 2");
        box_->addInput(input_mask_b_);

        output_img_ = new ConnectorOut(box_, 0);
        box_->addOutput(output_img_);

        insert(layout);
    }
}

void ImageCombiner::messageArrived(ConnectorIn* source)
{
    if(source == input_img_a_) {
        has_img_a = true;
    } else if(source == input_mask_a_) {
        has_mask_a = true;
    } else if(source == input_img_b_) {
        has_img_b = true;
    } else if(source == input_mask_b_) {
        has_mask_b = true;
    }

    if(!input_mask_a_->isConnected()) {
        has_mask_a = true;
    }
    if(!input_mask_b_->isConnected()) {
        has_mask_b = true;
    }


    if(!has_mask_a || !has_img_a || !has_mask_b || !has_img_b) {
        return;
    }

    has_img_a = false;
    has_mask_a = false;
    has_img_b = false;
    has_mask_b = false;

    CvMatMessage::Ptr img_msg_a = boost::dynamic_pointer_cast<CvMatMessage> (input_img_a_->getMessage());
    CvMatMessage::Ptr mask_msg_a = boost::dynamic_pointer_cast<CvMatMessage> (input_mask_a_->getMessage());
    CvMatMessage::Ptr img_msg_b = boost::dynamic_pointer_cast<CvMatMessage> (input_img_b_->getMessage());
    CvMatMessage::Ptr mask_msg_b = boost::dynamic_pointer_cast<CvMatMessage> (input_mask_b_->getMessage());

    if(img_msg_a.get() && !img_msg_a->value.empty() && img_msg_b.get() && !img_msg_b->value.empty()) {
        if(!mask_msg_a.get()) {
            mask_msg_a.reset(new CvMatMessage);
        }
        if(!mask_msg_b.get()) {
            mask_msg_b.reset(new CvMatMessage);
        }

        CvMatMessage::Ptr img_msg_result(new CvMatMessage);
        img_msg_result->value = combine(img_msg_a->value, mask_msg_a->value, img_msg_b->value, mask_msg_b->value);
        output_img_->publish(img_msg_result);
    }
}

void ImageCombiner::insert(QBoxLayout* layout)
{

}
