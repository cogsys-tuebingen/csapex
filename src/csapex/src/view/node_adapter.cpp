/// HEADER
#include <csapex/view/node_adapter.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/model/node_worker.h>

/// PROJECT
#include <utils_param/range_parameter.h>
#include <utils_param/value_parameter.h>
#include <utils_param/set_parameter.h>
#include <utils_param/path_parameter.h>
#include <utils_param/trigger_parameter.h>
#include <utils_param/color_parameter.h>

/// SYSTEM
#include <boost/bind.hpp>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QFileDialog>
#include <QColorDialog>
#include <iomanip>

using namespace csapex;

NodeAdapter::NodeAdapter()
    : bridge(this), layout_(NULL), is_gui_setup_(false)
{
    QObject::connect(&bridge, SIGNAL(rebuild()), &bridge, SLOT(rebuildEvent()));
}

NodeAdapter::~NodeAdapter()
{
    clear();
}

void NodeAdapter::clear()
{
    Q_FOREACH(const boost::signals2::connection& c, connections) {
        c.disconnect();
    }
    connections.clear();

    Q_FOREACH(QObject* cb, callbacks) {
        qt_helper::Call* call = dynamic_cast<qt_helper::Call*>(cb);
        if(call) {
            call->disconnect();
        }
        cb->deleteLater();
    }
    callbacks.clear();
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

void NodeAdapter::setupUiAgain()
{
    bridge.triggerRebuild();
}

void NodeAdapter::doSetupUi(QBoxLayout *layout)
{
    layout_ = layout;
    if(!is_gui_setup_) {
        is_gui_setup_ = true;
        setupUi(layout_);

        guiChanged();
    }
}

namespace {
void updateColor(param::ColorParameter *p)
{
    std::vector<int> c = p->value();
    QColor init(c[0], c[1], c[2]);
    QColor color = QColorDialog::getColor(init);
    if (color.isValid()) {
        std::vector<int> v(3);
        v[0] = color.red();
        v[1] = color.green();
        v[2] = color.blue();
        p->set(v);
    }
}

QString toColorSS(const std::vector<int>& v) {
    std::stringstream ss;
    ss << "QPushButton {";
    ss << "background-color: #" << std::hex << std::setfill('0');
    ss << std::setw(2) << v[0];
    ss << std::setw(2) << v[1];
    ss << std::setw(2) << v[2];
    ss << std::dec << ";";
    ss << "}";

    return QString::fromStdString(ss.str());
}
}


void NodeAdapter::setupUi(QBoxLayout * layout)
{
    bool is_update = layout->count() > 0;
    std::cout << (is_update ? "rebuild" : "init") << std::endl;

    QtHelper::clearLayout(layout_);
    clear();

    std::vector<param::Parameter::Ptr> params = node_->getParameters();
    Q_FOREACH(param::Parameter::Ptr parameter, params) {
        parameter_enabled(*parameter).disconnect_all_slots();
        parameter_enabled(*parameter).connect(boost::bind(&NodeAdapter::setupUiAgain, this));

        if(!parameter->isEnabled()) {
            continue;
        }

        std::string name = parameter->name();

        param::TriggerParameter::Ptr trigger_p = boost::dynamic_pointer_cast<param::TriggerParameter> (parameter);
        if(trigger_p) {
            QPushButton* btn = new QPushButton(trigger_p->name().c_str());

            QHBoxLayout* sub = new QHBoxLayout;
            sub->addWidget(btn);
            layout->addLayout(QtHelper::wrap(name, sub));

            boost::function<void()> cb = boost::bind(&param::TriggerParameter::trigger, trigger_p.get());
            qt_helper::Call* call_trigger = new qt_helper::Call(cb);
            callbacks.push_back(call_trigger);

            QObject::connect(btn, SIGNAL(clicked()), call_trigger, SLOT(call()));

            continue;
        }

        param::ColorParameter::Ptr color_p = boost::dynamic_pointer_cast<param::ColorParameter> (parameter);
        if(color_p) {
            QPushButton* btn = new QPushButton;

            btn->setStyleSheet(toColorSS(color_p->value()));

            QHBoxLayout* sub = new QHBoxLayout;
            sub->addWidget(btn);
            layout->addLayout(QtHelper::wrap(name, sub));

            // ui callback
            boost::function<void()> cb = boost::bind(&updateColor, color_p.get());
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);
            QObject::connect(btn, SIGNAL(clicked()), call, SLOT(call()));

            // model change -> ui
            boost::function<std::vector<int>()> readColor = boost::bind(&param::ColorParameter::value, color_p.get());
            boost::function<QString()> readSS = boost::bind(&toColorSS, boost::bind(readColor));
            boost::function<void(param::Parameter*)> up = boost::bind(&QPushButton::setStyleSheet, btn, boost::bind(readSS));

            connections.push_back(parameter_changed(*color_p).connect(up));
            continue;
        }

        param::PathParameter::Ptr path_p = boost::dynamic_pointer_cast<param::PathParameter> (parameter);
        if(path_p) {
            QLineEdit* path = new QLineEdit;
            QPushButton* open = new QPushButton("open");

            QHBoxLayout* sub = new QHBoxLayout;

            sub->addWidget(path);
            sub->addWidget(open);

            layout->addLayout(QtHelper::wrap(name, sub));

            // ui change -> model
            boost::function<std::string(const QString&)> qstring2stdstring = boost::bind(&QString::toStdString, _1);
            boost::function<void()> cb = boost::bind(&NodeAdapter::updateParam<std::string>, this, name, boost::bind(qstring2stdstring, boost::bind(&QLineEdit::text, path)));
            qt_helper::Call* call_set_path = new qt_helper::Call(cb);
            callbacks.push_back(call_set_path);


            boost::function<void()> cb_open = boost::bind(&NodeAdapter::updateParam<std::string>, this, name,
                                                          boost::bind(qstring2stdstring, boost::bind(&QFileDialog::getOpenFileName,
                                                                                                     (QWidget*) 0, "Input", "", "All files (*.*)", (QString*) 0, (QFlags<QFileDialog::Option>) 0)));
            qt_helper::Call* call_open = new qt_helper::Call(cb_open);
            callbacks.push_back(call_open);

            // model change -> ui
            boost::function<QString(const std::string&)> stdstring2qstring = boost::bind(&QString::fromStdString, _1);
            boost::function<void(const std::string&)> set = boost::bind(&QLineEdit::setText, path, boost::bind(stdstring2qstring, _1));
            connections.push_back(parameter_changed(*path_p).connect(boost::bind(&NodeAdapter::updateUi<std::string>, this, _1, set)));

            QObject::connect(path, SIGNAL(returnPressed()), call_set_path, SLOT(call()));
            QObject::connect(open, SIGNAL(clicked()), call_open, SLOT(call()));

            continue;
        }

        param::ValueParameter::Ptr value_p = boost::dynamic_pointer_cast<param::ValueParameter> (parameter);
        if(value_p) {
            if(value_p->is<std::string>()) {
                QLineEdit* txt_ = new QLineEdit;
                QPushButton* send = new QPushButton("set");

                QHBoxLayout* sub = new QHBoxLayout;

                sub->addWidget(txt_);
                sub->addWidget(send);

                layout->addLayout(QtHelper::wrap(name, sub));

                // ui change -> model
                boost::function<std::string(const QString&)> qstring2stdstring = boost::bind(&QString::toStdString, _1);
                boost::function<void()> cb = boost::bind(&NodeAdapter::updateParam<std::string>, this, name, boost::bind(qstring2stdstring, boost::bind(&QLineEdit::text, txt_)));
                qt_helper::Call* call = new qt_helper::Call(cb);
                callbacks.push_back(call);

                // model change -> ui
                boost::function<QString(const std::string&)> stdstring2qstring = boost::bind(&QString::fromStdString, _1);
                boost::function<void(const std::string&)> set = boost::bind(&QLineEdit::setText, txt_, boost::bind(stdstring2qstring, _1));
                connections.push_back(parameter_changed(*value_p).connect(boost::bind(&NodeAdapter::updateUi<std::string>, this, _1, set)));

                QObject::connect(txt_, SIGNAL(returnPressed()), call, SLOT(call()));
                QObject::connect(send, SIGNAL(clicked()), call, SLOT(call()));

            } else {
                layout->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
            }
            continue;
        }

        param::RangeParameter::Ptr range_p = boost::dynamic_pointer_cast<param::RangeParameter> (parameter);
        if(range_p) {
            if(range_p->is<int>()) {
                QSlider* slider = QtHelper::makeSlider(layout, name , range_p->as<int>(), range_p->min<int>(), range_p->max<int>());
                slider->setValue(range_p->as<int>());

                // ui change -> model
                boost::function<void()> cb = boost::bind(&NodeAdapter::updateParam<int>, this, name, boost::bind(&QSlider::value, slider));
                qt_helper::Call* call = new qt_helper::Call(cb);
                callbacks.push_back(call);

                // model change -> ui
                boost::function<void(int)> set = boost::bind(&QSlider::setValue, slider, _1);
                connections.push_back(parameter_changed(*range_p).connect(boost::bind(&NodeAdapter::updateUi<int>, this, _1, set)));

                QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

            } else if(range_p->is<double>()) {
                if(range_p->name() == "resolution") {
                    std::cout << "res: " << range_p->as<double>() << std::endl;
                }
                QDoubleSlider* slider = QtHelper::makeDoubleSlider(layout, name , range_p->as<double>(), range_p->min<double>(), range_p->max<double>(), range_p->step<double>());
                slider->setDoubleValue(range_p->as<double>());

                // ui change -> model
                boost::function<void()> cb = boost::bind(&NodeAdapter::updateParam<double>, this, name, boost::bind(&QDoubleSlider::doubleValue, slider));
                qt_helper::Call* call = new qt_helper::Call(cb);
                callbacks.push_back(call);

                // model change -> ui
                boost::function<void(double)> set = boost::bind(&QDoubleSlider::setDoubleValue, slider, _1);
                connections.push_back(parameter_changed(*range_p).connect(boost::bind(&NodeAdapter::updateUi<double>, this, _1, set)));

                QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

            } else if(range_p->is<bool>()) {
                QCheckBox* box = new QCheckBox;
                box->setChecked(range_p->as<bool>());

                layout->addLayout(QtHelper::wrap(name, box));

                // ui change -> model
                boost::function<void()> cb = boost::bind(&NodeAdapter::updateParam<bool>, this, name, boost::bind(&QCheckBox::isChecked, box));
                qt_helper::Call* call = new qt_helper::Call(cb);
                callbacks.push_back(call);

                // model change -> ui
                boost::function<void(bool)> set = boost::bind(&QCheckBox::setChecked, box, _1);
                connections.push_back(parameter_changed(*range_p).connect(boost::bind(&NodeAdapter::updateUi<bool>, this, _1, set)));

                QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

            } else {
                layout->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
            }
            continue;
        }

        param::SetParameter::Ptr set_p = boost::dynamic_pointer_cast<param::SetParameter> (parameter);
        if(set_p) {
            QComboBox* combo = new QComboBox;
            int current = 0;
            std::string selected = set_p->getName();
            for(int i = 0; i < set_p->noParameters(); ++i) {
                std::string str = set_p->getName(i);
                combo->addItem(QString::fromStdString(str));

                if(str == selected) {
                    current = i;
                }
            }
            combo->setCurrentIndex(current);
            layout->addLayout(QtHelper::wrap(name, combo));

            // ui change -> model
            boost::function<std::string(const QString&)> qstring2stdstring = boost::bind(&QString::toStdString, _1);
            boost::function<void()> cb = boost::bind(&NodeAdapter::updateParamSet, this, name, boost::bind(qstring2stdstring, boost::bind(&QComboBox::currentText, combo)));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            // model change -> ui
            boost::function<QString(const std::string&)> stdstring2qstring = boost::bind(&QString::fromStdString, _1);
            boost::function<int(const QString&)> txt2idx = boost::bind(&QComboBox::findData, combo, _1, Qt::DisplayRole, static_cast<Qt::MatchFlags>(Qt::MatchExactly|Qt::MatchCaseSensitive));
            boost::function<void(const QString&)> select = boost::bind(&QComboBox::setCurrentIndex, combo, boost::bind(txt2idx, _1));
            boost::function<void(const std::string&)> set = boost::bind(select, boost::bind(stdstring2qstring, _1));
            connections.push_back(parameter_changed(*set_p).connect(boost::bind(&NodeAdapter::updateUiSet, this, _1, set)));

            QObject::connect(combo, SIGNAL(currentIndexChanged(QString)), call, SLOT(call()));
            continue;
        }
    }
}

template <typename T>
void NodeAdapter::updateParam(const std::string& name, T value)
{
    param::Parameter* p = node_->getParameter(name).get();
    p->set<T>(value);
    guiChanged();
}

void NodeAdapter::updateParamSet(const std::string& name, const std::string& value)
{
    param::Parameter::Ptr parameter = node_->getParameter(name);
    param::SetParameter::Ptr set_p = boost::dynamic_pointer_cast<param::SetParameter> (parameter);
    if(set_p) {
        set_p->setByName(value);
        guiChanged();
    }
}

template <typename T>
void NodeAdapter::updateUi(const param::Parameter* p, boost::function<void(T)> setter)
{
    setter(node_->param<T>(p->name()));
}


void NodeAdapter::updateUiSet(const param::Parameter *p, boost::function<void (const std::string &)> setter)
{
    const param::SetParameter* set_p = dynamic_cast<const param::SetParameter*> (p);
    if(set_p) {
        setter(set_p->getName());
    }
}
void NodeAdapter::updateDynamicGui(QBoxLayout *)
{

}

void NodeAdapter::modelChangedEvent()
{

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

void NodeAdapterBridge::rebuildEvent()
{
    parent_->setupUi(parent_->layout_);
}

void NodeAdapterBridge::triggerGuiChanged()
{
    Q_EMIT guiChanged();
}

void NodeAdapterBridge::triggerRebuild()
{
    std::cout << "trigger" << std::endl;
    Q_EMIT rebuild();
}
