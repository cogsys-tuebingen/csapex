/// HEADER
#include "extract_keypoints.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>

/// PROJECT
#include <utils/extractor.h>
#include <utils/extractor_factory.h>
#include <utils/extractor_manager.h>
#include <utils_param/range_parameter.h>
#include <utils_param/value_parameter.h>
#include <utils_param/io.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <QFrame>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ExtractKeypoints, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExtractKeypoints::ExtractKeypoints()
    : selection_key(NULL), change(false)
{
    addTag(Tag::get("Features"));
}

void ExtractKeypoints::allConnectorsArrived()
{
    if(!extractor) {
        setError(true, "no extractor set");
        return;
    }

    if(change) {
        return;
    }

    setError(false);

    CvMatMessage::Ptr img_msg = in_img->getMessage<CvMatMessage>();

    KeypointMessage::Ptr key_msg(new KeypointMessage);

    {
        QMutexLocker lock(&extractor_mutex);
        if(in_mask->isConnected()) {
            CvMatMessage::Ptr mask_msg = in_mask->getMessage<CvMatMessage>();

            extractor->extractKeypoints(img_msg->value, mask_msg->value, key_msg->value);

        } else {
            extractor->extractKeypoints(img_msg->value, cv::Mat(), key_msg->value);
        }
    }

    out_key->publish(key_msg);
}


void ExtractKeypoints::fill(QBoxLayout* layout)
{
    if(selection_key == NULL) {
        setSynchronizedInputs(true);

        in_img = addInput<CvMatMessage>("Image");
        in_mask = addInput<CvMatMessage>("Mask", true);

        out_key = addOutput<csapex::connection_types::KeypointMessage>("Keypoints");

        ExtractorManager& manager = ExtractorManager::instance();

        selection_key = new QComboBox;
        typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
        Q_FOREACH(Pair fc, manager.featureDetectors()) {
            std::string key = fc.second.getType();
            selection_key->addItem(key.c_str());

            state.params[key] = ExtractorManager::instance().featureDetectorParameters(key);
        }
        layout->addLayout(QtHelper::wrap("Keypoint", selection_key));

        opt = new QFrame;
        opt->setLayout(new QVBoxLayout);
        layout->addWidget(opt);

        QObject::connect(selection_key, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));

        QObject::connect(this, SIGNAL(started()), this, SIGNAL(modelChanged()));
    }
}

void ExtractKeypoints::updateDynamicGui(QBoxLayout *layout)
{
    update(0);
}

template <typename T>
void ExtractKeypoints::updateParam(const std::string& name, T value)
{
    BOOST_FOREACH(param::Parameter::Ptr& para, state.params[state.key]) {
        if(para->name() == name) {
            para->set<T>(value);

            change = true;
            Q_EMIT guiChanged();

            return;
        }
    }
}

void ExtractKeypoints::update(int slot)
{
    state.key = selection_key->currentText().toStdString();

    QtHelper::clearLayout(opt->layout());
    QBoxLayout* layout = dynamic_cast<QBoxLayout*> (opt->layout());
    assert(layout);

    Q_FOREACH(QObject* cb, callbacks) {
        delete cb;
    }
    callbacks.clear();

    Q_FOREACH(const param::Parameter::Ptr& p, state.params[state.key]) {
        std::string name = p->name();

        param::ValueParameter::Ptr value = boost::dynamic_pointer_cast<param::ValueParameter> (p);
        if(value) {
            if(value->is<bool>()) {
                QCheckBox* box = new QCheckBox;
                box->setChecked(value->as<bool>());

                layout->addLayout(QtHelper::wrap(name, box));

                boost::function<void()> cb = boost::bind(&ExtractKeypoints::updateParam<bool>, this, name, boost::bind(&QCheckBox::isChecked, box));
                qt_helper::Call* call = new qt_helper::Call(cb);
                callbacks.push_back(call);

                QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

            }
            continue;
        }

        param::RangeParameter::Ptr range = boost::dynamic_pointer_cast<param::RangeParameter> (p);
        if(range->is<int>()) {
            QSlider* slider = QtHelper::makeSlider(layout, name , range->as<int>(), range->min<int>(), range->max<int>());
            slider->setValue(range->as<int>());

            boost::function<void()> cb = boost::bind(&ExtractKeypoints::updateParam<int>, this, name, boost::bind(&QSlider::value, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(range->is<double>()) {
            QDoubleSlider* slider = QtHelper::makeDoubleSlider(layout, name , range->as<double>(), range->min<double>(), range->max<double>(), range->step<double>());
            slider->setDoubleValue(range->as<double>());

            boost::function<void()> cb = boost::bind(&ExtractKeypoints::updateParam<double>, this, name, boost::bind(&QDoubleSlider::doubleValue, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else {
            opt->layout()->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
        }

    }


    update();
}

void ExtractKeypoints::updateModel()
{
    if(change) {
        change = false;
        update();
    }
}

void ExtractKeypoints::update()
{
    Extractor::Ptr next = ExtractorFactory::create(state.key, "", param::StaticParameterProvider(state.params[state.key]));

    QMutexLocker lock(&extractor_mutex);
    extractor = next;
}

Memento::Ptr ExtractKeypoints::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ExtractKeypoints::setState(Memento::Ptr memento)
{
    boost::shared_ptr<ExtractKeypoints::State> m = boost::dynamic_pointer_cast<ExtractKeypoints::State> (memento);
    assert(m.get());

    //    state = *m;
    state.key = m->key;

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

    if(selection_key) {
        int slot = 0;
        for(int i = 0, n = selection_key->count(); i < n; ++i) {
            if(selection_key->itemText(i).toStdString() == state.key) {
                slot = i;
                break;
            }
        }
        selection_key->setCurrentIndex(slot);
    }
}


void ExtractKeypoints::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "key" << YAML::Value << key;
    out << YAML::Key << "params" << YAML::Value << params;
}
void ExtractKeypoints::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("params")) {
        node["params"] >> params;
    }
    if(node.FindValue("key")) {
        node["key"] >> key;
    }
}
