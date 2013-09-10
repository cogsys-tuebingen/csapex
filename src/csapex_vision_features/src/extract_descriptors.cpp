/// HEADER
#include "extract_descriptors.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

/// PROJECT
#include <utils/extractor.h>
#include <utils/extractor_factory.h>
#include <utils/extractor_manager.h>
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <QFrame>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::ExtractDescriptors, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

ExtractDescriptors::ExtractDescriptors()
    : selection_des(NULL), change(false), has_img(false), has_kp(false)
{
    addTag(Tag::get("Features"));
}

void ExtractDescriptors::messageArrived(ConnectorIn *source)
{
    if(source == in_img) {
        has_img = true;
    } else if(source == in_key) {
        has_kp = true;
    }

    if(!extractor) {
        setError(true, "no extractor set");
        return;
    }

    if(has_img && has_kp) {
        setError(false);

        has_img = false;
        has_kp = false;

        ConnectionType::Ptr msg = in_img->getMessage();
        CvMatMessage::Ptr img_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

        DescriptorMessage::Ptr des_msg(new DescriptorMessage);

        {
            QMutexLocker lock(&extractor_mutex);
            ConnectionType::Ptr msg = in_key->getMessage();
            KeypointMessage::Ptr key_msg = boost::dynamic_pointer_cast<KeypointMessage>(msg);

            extractor->extractDescriptors(img_msg->value, key_msg->value, des_msg->value);
        }

        out_des->publish(des_msg);
    }
}


void ExtractDescriptors::fill(QBoxLayout* layout)
{
    if(selection_des == NULL) {
        in_img = new ConnectorIn(box_, 0);
        in_img->setLabel("Image");
        box_->addInput(in_img);
        in_key = new ConnectorIn(box_, 1);
        in_key->setType(csapex::connection_types::KeypointMessage::make());
        in_key->setLabel("Keypoints");
        box_->addInput(in_key);

        out_des = new ConnectorOut(box_, 0);
        out_des->setLabel("Descriptors");
        box_->addOutput(out_des);

        ExtractorManager& manager = ExtractorManager::instance();

        selection_des = new QComboBox;
        typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
        foreach(Pair fc, manager.descriptorExtractors()) {
            selection_des->addItem(fc.second.getType().c_str());
        }
        layout->addLayout(QtHelper::wrap("Descriptor", selection_des));

        opt = new QFrame;
        opt->setLayout(new QVBoxLayout);
        layout->addWidget(opt);


        QObject::connect(selection_des, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));

        update(0);
    }
}

template <typename T>
void ExtractDescriptors::updateParam(const std::string& name, T value)
{
    BOOST_FOREACH(vision::Parameter& para, params) {
        if(para.name() == name) {
            para.set<T>(value);

            change = true;
            Q_EMIT guiChanged();

            return;
        }
    }
}

void ExtractDescriptors::update(int slot)
{
    des = selection_des->currentText().toStdString();

    QtHelper::clearLayout(opt->layout());
    QBoxLayout* layout = dynamic_cast<QBoxLayout*> (opt->layout());
    assert(layout);

    params.clear();
    foreach(QObject* cb, callbacks) {
        delete cb;
    }
    callbacks.clear();

    params = ExtractorManager::instance().featureDescriptorParameters(des);

    foreach(const vision::Parameter& para, params) {
        if(para.is<int>()) {
            QSlider* slider = QtHelper::makeSlider(layout, para.name(), para.def<int>(), para.min<int>(), para.max<int>());

            boost::function<void()> cb = boost::bind(&ExtractDescriptors::updateParam<int>, this, para.name(), boost::bind(&QSlider::value, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else {
            opt->layout()->addWidget(new QLabel(para.name().c_str()));
        }
    }


    update();
}

void ExtractDescriptors::updateModel()
{
    if(change) {
        change = false;
        update();
    }
}

void ExtractDescriptors::update()
{
    QMutexLocker lock(&extractor_mutex);
    extractor = ExtractorFactory::create("", des, vision::StaticParameterProvider(params));
}
