/// HEADER
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <boost/bind.hpp>
#include <QCheckBox>

using namespace csapex;

NodeAdapter::NodeAdapter()
    : bridge(this), is_gui_setup_(false)
{

}

NodeAdapter::~NodeAdapter()
{

}

void NodeAdapter::setNode(Node *node)
{
    node_ = node;

    QObject::connect(node, SIGNAL(modelChanged()), &bridge, SLOT(modelChangedEvent()));
}

Node* NodeAdapter::getNode()
{
    return node_;
}

void NodeAdapter::doSetupUi(QBoxLayout *layout)
{
    if(!is_gui_setup_) {
        is_gui_setup_ = true;
        setupUi(layout);

        guiChanged();
    }
}

void NodeAdapter::setupUi(QBoxLayout * layout)
{
    std::vector<param::Parameter::Ptr> params = node_->getParameters();
    foreach(param::Parameter::Ptr p, params) {
        std::string name = p->name();

        if(p->is<int>()) {
            QSlider* slider = QtHelper::makeSlider(layout, name , p->as<int>(), p->min<int>(), p->max<int>());
            slider->setValue(p->as<int>());

            // ui change -> model
            boost::function<void()> cb = boost::bind(&NodeAdapter::updateParam<int>, this, name, boost::bind(&QSlider::value, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            // model change -> ui
            boost::function<void(int)> set = boost::bind(&QSlider::setValue, slider, _1);
            p->parameter_changed->connect(boost::bind(&NodeAdapter::updateUi<int>, this, _1, set));

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(p->is<double>()) {
            QDoubleSlider* slider = QtHelper::makeDoubleSlider(layout, name , p->as<double>(), p->min<double>(), p->max<double>(), p->step<double>());
            slider->setDoubleValue(p->as<double>());

            // ui change -> model
            boost::function<void()> cb = boost::bind(&NodeAdapter::updateParam<double>, this, name, boost::bind(&QDoubleSlider::doubleValue, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            // model change -> ui
            boost::function<void(double)> set = boost::bind(&QDoubleSlider::setDoubleValue, slider, _1);
            p->parameter_changed->connect(boost::bind(&NodeAdapter::updateUi<double>, this, _1, set));

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(p->is<bool>()) {
            QCheckBox* box = new QCheckBox;
            box->setChecked(p->as<bool>());

            layout->addLayout(QtHelper::wrap(name, box));

            // ui change -> model
            boost::function<void()> cb = boost::bind(&NodeAdapter::updateParam<bool>, this, name, boost::bind(&QCheckBox::isChecked, box));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            // model change -> ui
            boost::function<void(bool)> set = boost::bind(&QCheckBox::setChecked, box, _1);
            p->parameter_changed->connect(boost::bind(&NodeAdapter::updateUi<bool>, this, _1, set));

            QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

        } else {
            layout->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
        }
    }
}

template <typename T>
void NodeAdapter::updateParam(const std::string& name, T value)
{
    node_->getParameter(name)->set<T>(value);
    guiChanged();
}

template <typename T>
void NodeAdapter::updateUi(const param::Parameter* p, boost::function<void(T)> setter)
{
    setter(node_->param<T>(p->name()));
}

void NodeAdapter::updateDynamicGui(QBoxLayout *)
{

}

void NodeAdapter::modelChangedEvent()
{
//    std::vector<param::Parameter::Ptr> params = node_->getParameters();
//    foreach(param::Parameter::Ptr p, params) {
//        p->name()
//    }
}

void NodeAdapter::guiChanged()
{
    bridge.triggerGuiChanged();
}

NodeAdapterBridge::NodeAdapterBridge(NodeAdapter *parent)
    : parent_(parent)
{

}

void NodeAdapterBridge::modelChangedEvent()
{
    parent_->modelChangedEvent();
}

void NodeAdapterBridge::triggerGuiChanged()
{
    Q_EMIT guiChanged();
}
