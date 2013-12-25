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

/// SYSTEM
#include <boost/bind.hpp>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QFileDialog>

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
    foreach(param::Parameter::Ptr parameter, params) {
        std::string name = parameter->name();

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
            node_->getNodeWorker()->addParameterCallback(path_p, boost::bind(&NodeAdapter::updateUi<std::string>, this, _1, set));

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
                node_->getNodeWorker()->addParameterCallback(value_p, boost::bind(&NodeAdapter::updateUi<std::string>, this, _1, set));

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
                node_->getNodeWorker()->addParameterCallback(range_p, boost::bind(&NodeAdapter::updateUi<int>, this, _1, set));

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
                node_->getNodeWorker()->addParameterCallback(range_p, boost::bind(&NodeAdapter::updateUi<double>, this, _1, set));

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
                node_->getNodeWorker()->addParameterCallback(range_p, boost::bind(&NodeAdapter::updateUi<bool>, this, _1, set));

                QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

            } else {
                layout->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
            }
            continue;
        }

        param::SetParameter::Ptr set_p = boost::dynamic_pointer_cast<param::SetParameter> (parameter);
        if(set_p) {
            QComboBox* combo = new QComboBox;
            for(int i = 0; i < set_p->noParameters(); ++i) {
                std::string str = set_p->getName(i);
                combo->addItem(QString::fromStdString(str));
            }
            combo->setCurrentIndex(0);
            layout->addLayout(QtHelper::wrap(name, combo));

            // ui change -> model
            boost::function<std::string(const QString&)> qstring2stdstring = boost::bind(&QString::toStdString, _1);
            boost::function<void()> cb = boost::bind(&NodeAdapter::updateParamSet, this, name, boost::bind(qstring2stdstring, boost::bind(&QComboBox::currentText, combo)));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            // model change -> ui
            boost::function<QString(const std::string&)> stdstring2qstring = boost::bind(&QString::fromStdString, _1);
            boost::function<int(const QString&)> txt2idx = boost::bind(&QComboBox::findData, combo, _1, Qt::UserRole, static_cast<Qt::MatchFlags>(Qt::MatchExactly|Qt::MatchCaseSensitive));
            boost::function<void(const QString&)> select = boost::bind(&QComboBox::setCurrentIndex, combo, boost::bind(txt2idx, _1));
            boost::function<void(const std::string&)> set = boost::bind(select, boost::bind(stdstring2qstring, _1));
          // node_->getNodeWorker()->addParameterCallback(set_p, boost::bind(&NodeAdapter::updateUi<std::string>, this, _1, set));

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
