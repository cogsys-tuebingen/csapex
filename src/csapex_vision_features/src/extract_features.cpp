/// HEADER
#include "extract_features.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

/// PROJECT
#include <utils/extractor.h>
#include <utils/extractor_factory.h>
#include <utils/extractor_manager.h>
#include <csapex/box.h>
#include <csapex/connector_out.h>
#include <csapex/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::ExtractFeatures, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

ExtractFeatures::ExtractFeatures()
    : selection_key(NULL), has_img(false), has_mask(false)
{
    addTag(Tag::get("Features"));
}

void ExtractFeatures::messageArrived(ConnectorIn *source)
{
    if(source == in_img) {
        has_img = true;
    } else if(source == in_mask) {
        has_mask = true;
    }

    if(!extractor) {
        setError(true, "no extractor set");
        return;
    }

    bool use_mask = in_mask->isConnected();
    if(has_img && (has_mask || !use_mask)) {
        setError(false);

        has_img = false;
        has_mask = false;

        ConnectionType::Ptr msg = in_img->getMessage();
        CvMatMessage::Ptr img_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

        KeypointMessage::Ptr key_msg(new KeypointMessage);
        DescriptorMessage::Ptr des_msg(new DescriptorMessage);

        {
            QMutexLocker lock(&extractor_mutex);
            if(use_mask) {
                ConnectionType::Ptr msg = in_mask->getMessage();
                CvMatMessage::Ptr mask_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

                extractor->extract(img_msg->value, mask_msg->value, key_msg->value, des_msg->value);

            } else {
                extractor->extract(img_msg->value, cv::Mat(), key_msg->value, des_msg->value);
            }
        }

        out_key->publish(key_msg);
        out_des->publish(des_msg);
    }
}


void ExtractFeatures::fill(QBoxLayout* layout)
{
    if(selection_key == NULL) {
        in_img = new ConnectorIn(box_, 0);
        in_img->setLabel("Image");
        box_->addInput(in_img);
        in_mask = new ConnectorIn(box_, 1);
        in_mask->setLabel("Mask (opt.)");
        box_->addInput(in_mask);

        out_key = new ConnectorOut(box_, 0);
        out_key->setType(csapex::connection_types::KeypointMessage::make());
        out_key->setLabel("Keypoints");
        box_->addOutput(out_key);

        out_des = new ConnectorOut(box_, 1);
        out_des->setLabel("Descriptors");
        box_->addOutput(out_des);

        ExtractorManager manager;

        selection_key = new QComboBox;
        typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
        foreach(Pair fc, manager.featureDetectors()) {
            selection_key->addItem(fc.second.getType().c_str());
        }
        layout->addLayout(QtHelper::wrap("Keypoint", selection_key));

        selection_des = new QComboBox;
        typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
        foreach(Pair fc, manager.descriptorExtractors()) {
            selection_des->addItem(fc.second.getType().c_str());
        }
        layout->addLayout(QtHelper::wrap("Descriptor", selection_des));


        QObject::connect(selection_key, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));
        QObject::connect(selection_des, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));

        update(0);
    }
}

void ExtractFeatures::update(int slot)
{
    std::string key = selection_key->currentText().toStdString();
    std::string des = selection_des->currentText().toStdString();

    QMutexLocker lock(&extractor_mutex);
    extractor = ExtractorFactory::create(key, des);
}
