/// HEADER
#include "object_detection.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

// Register the class
PLUGINLIB_EXPORT_CLASS(csapex::ObjectDetection, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

ObjectDetection::ObjectDetection()
    : in_a_(NULL),
      in_b_(NULL),
      in_c_(NULL),
      in_d_(NULL),
      out_(NULL),
      btn_(NULL),
      has_a_(false),
      has_b_(false),
      has_c_(false),
      has_d_(false),
      publish_a_(true),
      container_sliders_(NULL)
{
    // Insert this plug-in into the "Vision" category.
    // Create the category, if it doesn't exist.
    Tag::createIfNotExists("Vision");
    addTag(Tag::get("Vision"));
    minHessian = 400;
}

void ObjectDetection::fill(QBoxLayout *layout)
{
    // Connector for the first input image (sub id = 0)
    in_a_ = new ConnectorIn(box_, 0);
    in_a_->setLabel("Object");
    in_a_->setType(connection_types::CvMatMessage::make());

    // Connector for the second input image (sub id = 1)
    in_b_ = new ConnectorIn(box_, 1);
    in_b_->setLabel("Mask 1");
    in_b_->setType(connection_types::CvMatMessage::make());

    // Connector for the second input image (sub id = 1)
    in_c_ = new ConnectorIn(box_, 2);
    in_c_->setLabel("Scene");
    in_c_->setType(connection_types::CvMatMessage::make());

    // Connector for the second input image (sub id = 1)
    in_d_ = new ConnectorIn(box_, 3);
    in_d_->setLabel("Mask 2");
    in_d_->setType(connection_types::CvMatMessage::make());

    // Connector for the output input image (sub id = 0)
    out_ = new ConnectorOut(box_, 0);
    out_->setLabel("output");
    out_->setType(connection_types::CvMatMessage::make());

    // Register the connectors
    box_->addInput(in_a_);
    box_->addInput(in_b_);
    box_->addInput(in_c_);
    box_->addInput(in_d_);
    box_->addOutput(out_);

    // Combobox to choose between different keydetectors
    detectorbox_ = new QComboBox();
    detectorbox_->addItem("SURF");
    detectorbox_->addItem("Dummy1");
    detectorbox_->addItem("Dummy2");
    layout->addLayout(QtHelper::wrap("Detector", detectorbox_));

    extractorbox_ = new QComboBox();
    extractorbox_->addItem("SURF");
    layout->addLayout(QtHelper::wrap("Extractor", extractorbox_));

    matcherbox_ = new QComboBox();
    matcherbox_->addItem("SURF");
    layout->addLayout(QtHelper::wrap("Matcher", matcherbox_));

    // Connect the button via signals to a private slot "buttonPressed"

    //Connect Combobox to Signal
    QObject::connect(detectorbox_, SIGNAL(currentIndexChanged(int)), this , SLOT(updateDetector()));

    Q_EMIT modelChanged();
}

void ObjectDetection::updateSliders(){
    minHessian = hessian_slider_->value();
}

void ObjectDetection::updateDetector(){
        Q_EMIT modelChanged();
}

void ObjectDetection::updateDynamicGui(QBoxLayout *layout){
    QVBoxLayout *internal_layout;
    internal_layout = new QVBoxLayout;

    if(detectorbox_->currentIndex() == 0){

    // Qslider for Minhessian value
    hessian_slider_ =  QtHelper::makeSlider(internal_layout,"minHessian",minHessian,1,5000);

    //Connect Slider to Value
    QObject::connect(hessian_slider_, SIGNAL( valueChanged(int) ), this, SLOT( updateSliders() ) );

    container_sliders_ = QtHelper::wrapLayout(internal_layout);

    layout->addWidget(container_sliders_);
    }
    else{
        if(container_sliders_ != NULL) {
            container_sliders_->deleteLater();
            container_sliders_ =  NULL;
        }
    }

}

void ObjectDetection::buttonPressed()
{
    // Update, which image to re-publish
    publish_a_ = !btn_->isChecked();

    // Update the UI
    btn_->setText(QString("Image ") + (publish_a_ ? "1" : "2"));
}

void ObjectDetection::messageArrived(ConnectorIn *source)
{
    // One of the two connectors has received a message, find out which
    if(source == in_a_) {
        has_a_ = true;
    } else if(source == in_c_) {
        has_c_ = true;
    }
    // Make sure that we have both images
    if(has_a_ && has_c_) {
        has_a_ = has_c_ = false;

        // Publish the selected image
        CvMatMessage::Ptr img_msg_a = boost::dynamic_pointer_cast<CvMatMessage> (in_a_->getMessage());
        CvMatMessage::Ptr mask_msg_a = boost::dynamic_pointer_cast<CvMatMessage> (in_b_->getMessage());
        CvMatMessage::Ptr img_msg_b = boost::dynamic_pointer_cast<CvMatMessage> (in_c_->getMessage());
        CvMatMessage::Ptr mask_msg_b = boost::dynamic_pointer_cast<CvMatMessage> (in_d_->getMessage());

        if(img_msg_a.get() && !img_msg_a->value.empty() && img_msg_b.get() && !img_msg_b->value.empty()) {
            if(!mask_msg_a.get()) {
                mask_msg_a.reset(new CvMatMessage);
            }
            if(!mask_msg_b.get()) {
                mask_msg_b.reset(new CvMatMessage);
            }
            Surfhomography surfhomo;
            CvMatMessage::Ptr img_msg_result(new CvMatMessage);
            img_msg_result->value = surfhomo.calculation(img_msg_a->value, img_msg_b->value, minHessian);
            out_->publish(img_msg_result);
        }

    }
}
