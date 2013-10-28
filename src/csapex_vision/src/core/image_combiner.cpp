/// HEADER
#include <csapex_vision/image_combiner.h>

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>


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
        input_img_a_ = addInput<CvMatMessage>("Image 1");
        input_mask_a_ = addInput<CvMatMessage>("Mask 1");
        input_img_b_ = addInput<CvMatMessage>("Image 2");
        input_mask_b_ = addInput<CvMatMessage>("Mask 2");

        output_img_ = addOutput<CvMatMessage>("Image");

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

    CvMatMessage::Ptr img_msg_a  = input_img_a_-> getMessage<CvMatMessage>();
    CvMatMessage::Ptr mask_msg_a = input_mask_a_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr img_msg_b  = input_img_b_-> getMessage<CvMatMessage>();
    CvMatMessage::Ptr mask_msg_b = input_mask_b_->getMessage<CvMatMessage>();

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
