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
    : selection_des(NULL), change(false)
{
    addTag(Tag::get("Features"));
}

void ExtractDescriptors::allConnectorsArrived()
{
    if(!extractor) {
        setError(true, "no extractor set");
        return;
    }

    setError(false);

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
        out_des->setType(csapex::connection_types::DescriptorMessage::make());
        box_->addOutput(out_des);

        ExtractorManager& manager = ExtractorManager::instance();

        selection_des = new QComboBox;
        typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
        foreach(Pair fc, manager.descriptorExtractors()) {
            std::string des = fc.second.getType();
            selection_des->addItem(fc.second.getType().c_str());

            state.params[des] = ExtractorManager::instance().featureDescriptorParameters(des);
        }
        layout->addLayout(QtHelper::wrap("Descriptor", selection_des));

        opt = new QFrame;
        opt->setLayout(new QVBoxLayout);
        layout->addWidget(opt);


        QObject::connect(selection_des, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));

        QObject::connect(box_, SIGNAL(placed()), this, SIGNAL(modelChanged()));
    }
}

void ExtractDescriptors::updateDynamicGui(QBoxLayout *layout)
{
    update(0);
}

template <typename T>
void ExtractDescriptors::updateParam(const std::string& name, T value)
{
    BOOST_FOREACH(vision::Parameter& para, state.params[state.des]) {
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
    state.des = selection_des->currentText().toStdString();

    QtHelper::clearLayout(opt->layout());
    QBoxLayout* layout = dynamic_cast<QBoxLayout*> (opt->layout());
    assert(layout);

    foreach(QObject* cb, callbacks) {
        delete cb;
    }
    callbacks.clear();

    foreach(const vision::Parameter& para, state.params[state.des]) {
        std::string name = para.name();

        if(para.is<int>()) {
            QSlider* slider = QtHelper::makeSlider(layout, name , para.as<int>(), para.min<int>(), para.max<int>());
            slider->setValue(para.as<int>());

            boost::function<void()> cb = boost::bind(&ExtractDescriptors::updateParam<int>, this, name, boost::bind(&QSlider::value, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(para.is<double>()) {
            QDoubleSlider* slider = QtHelper::makeDoubleSlider(layout, name , para.as<double>(), para.min<double>(), para.max<double>(), para.step<double>());
            slider->setDoubleValue(para.as<double>());

            boost::function<void()> cb = boost::bind(&ExtractDescriptors::updateParam<double>, this, name, boost::bind(&QDoubleSlider::doubleValue, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(para.is<bool>()) {
            QCheckBox* box = new QCheckBox;
            box->setChecked(para.as<bool>());

            layout->addLayout(QtHelper::wrap(name, box));

            boost::function<void()> cb = boost::bind(&ExtractDescriptors::updateParam<bool>, this, name, boost::bind(&QCheckBox::isChecked, box));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

        } else {
            opt->layout()->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
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
    Extractor::Ptr next = ExtractorFactory::create("", state.des, vision::StaticParameterProvider(state.params[state.des]));

    QMutexLocker lock(&extractor_mutex);
    extractor = next;
}

Memento::Ptr ExtractDescriptors::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ExtractDescriptors::setState(Memento::Ptr memento)
{
    boost::shared_ptr<ExtractDescriptors::State> m = boost::dynamic_pointer_cast<ExtractDescriptors::State> (memento);
    assert(m.get());

    //    state = *m;
    state.des = m->des;

    typedef std::pair<std::string, std::vector<vision::Parameter> > Pair;
    foreach(Pair pair, m->params) {
        foreach(const vision::Parameter& para, pair.second) {
            std::vector<vision::Parameter>& target = state.params[pair.first];
            BOOST_FOREACH(vision::Parameter& existing_param, target) {
                if(existing_param.name() == para.name()) {
                    existing_param.setFrom(para);
                }
            }
        }
    }

    int slot = 0;
    for(int i = 0, n = selection_des->count(); i < n; ++i) {
        if(selection_des->itemText(i).toStdString() == state.des) {
            slot = i;
            break;
        }
    }
    selection_des->setCurrentIndex(slot);
}


void ExtractDescriptors::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "des" << YAML::Value << des;
    out << YAML::Key << "params" << YAML::Value << params;
}
void ExtractDescriptors::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("params")) {
        node["params"] >> params;
    }
    if(node.FindValue("des")) {
        node["des"] >> des;
    }
}
