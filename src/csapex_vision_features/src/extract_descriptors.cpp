/// HEADER
#include "extract_descriptors.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

/// PROJECT
#include <utils/extractor.h>
#include <utils/extractor_factory.h>
#include <utils/extractor_manager.h>
#include <utils_param/range_parameter.h>
#include <utils_param/io.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <QFrame>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ExtractDescriptors, csapex::Node)

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

    CvMatMessage::Ptr img_msg = in_img->getMessage<CvMatMessage>();

    DescriptorMessage::Ptr des_msg(new DescriptorMessage);

    {
        QMutexLocker lock(&extractor_mutex);
        KeypointMessage::Ptr key_msg = in_key->getMessage<KeypointMessage>();

        extractor->extractDescriptors(img_msg->value, key_msg->value, des_msg->value);
    }

    out_des->publish(des_msg);
}


void ExtractDescriptors::fill(QBoxLayout* layout)
{
    if(selection_des == NULL) {
        setSynchronizedInputs(true);

        in_img = addInput<CvMatMessage>("Image");
        in_key = addInput<KeypointMessage>("Keypoints");

        out_des = addOutput<DescriptorMessage>("Descriptors");

        ExtractorManager& manager = ExtractorManager::instance();

        selection_des = new QComboBox;
        typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
        Q_FOREACH(Pair fc, manager.descriptorExtractors()) {
            std::string des = fc.second.getType();
            selection_des->addItem(fc.second.getType().c_str());

            state.params[des] = ExtractorManager::instance().featureDescriptorParameters(des);
        }
        layout->addLayout(QtHelper::wrap("Descriptor", selection_des));

        opt = new QFrame;
        opt->setLayout(new QVBoxLayout);
        layout->addWidget(opt);


        QObject::connect(selection_des, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));

        QObject::connect(this, SIGNAL(started()), this, SIGNAL(modelChanged()));
    }
}

void ExtractDescriptors::updateDynamicGui(QBoxLayout *layout)
{
    update(0);
}

template <typename T>
void ExtractDescriptors::updateParam(const std::string& name, T value)
{
    BOOST_FOREACH(param::Parameter::Ptr& para, state.params[state.des]) {
        if(para->name() == name) {
            para->set<T>(value);

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

    Q_FOREACH(QObject* cb, callbacks) {
        delete cb;
    }
    callbacks.clear();

    Q_FOREACH(const param::Parameter::Ptr& p, state.params[state.des]) {
        std::string name = p->name();

        param::RangeParameter::Ptr range = boost::dynamic_pointer_cast<param::RangeParameter> (p);

        if(range->is<int>()) {
            QSlider* slider = QtHelper::makeSlider(layout, name , range->as<int>(), range->min<int>(), range->max<int>());
            slider->setValue(range->as<int>());

            boost::function<void()> cb = boost::bind(&ExtractDescriptors::updateParam<int>, this, name, boost::bind(&QSlider::value, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(range->is<double>()) {
            QDoubleSlider* slider = QtHelper::makeDoubleSlider(layout, name , range->as<double>(), range->min<double>(), range->max<double>(), range->step<double>());
            slider->setDoubleValue(range->as<double>());

            boost::function<void()> cb = boost::bind(&ExtractDescriptors::updateParam<double>, this, name, boost::bind(&QDoubleSlider::doubleValue, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(range->is<bool>()) {
            QCheckBox* box = new QCheckBox;
            box->setChecked(range->as<bool>());

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
    Extractor::Ptr next = ExtractorFactory::create("", state.des, param::StaticParameterProvider(state.params[state.des]));

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

    typedef std::pair<std::string, std::vector<param::Parameter::Ptr> > Pair;
    Q_FOREACH(Pair pair, m->params) {
        Q_FOREACH(const param::Parameter::Ptr& para, pair.second) {
            std::vector<param::Parameter::Ptr>& target = state.params[pair.first];
            BOOST_FOREACH(param::Parameter::Ptr& existing_param, target) {
                if(existing_param->name() == para->name()) {
                    existing_param->setFrom(*para);
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
