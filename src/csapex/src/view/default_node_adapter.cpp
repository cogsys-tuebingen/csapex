/// HEADER
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include <csapex/utility/qt_helper.hpp>
#include <csapex/utility/q_signal_relay.h>
#include <csapex/model/node.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/view/port.h>
#include <csapex/view/widget_controller.h>
#include <csapex/model/node_worker.h>

/// PROJECT
#include <utils_param/range_parameter.h>
#include <utils_param/interval_parameter.h>
#include <utils_param/value_parameter.h>
#include <utils_param/set_parameter.h>
#include <utils_param/bitset_parameter.h>
#include <utils_param/path_parameter.h>
#include <utils_param/trigger_parameter.h>
#include <utils_param/color_parameter.h>

/// SYSTEM
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <iomanip>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QFileDialog>
#include <QColorDialog>
#include <QGroupBox>
#include <QScrollArea>
#include <QListWidget>
#include <QMouseEvent>
#include <QMenu>
#include <QAction>

using namespace csapex;
using namespace boost::lambda;
namespace bll = boost::lambda;

boost::arg<1> __1;
boost::arg<2> __2;

/// BRIDGE
DefaultNodeAdapterBridge::DefaultNodeAdapterBridge(DefaultNodeAdapter *parent)
    : parent_(parent)
{
    QObject::connect(this, SIGNAL(setupAdaptiveUiRequest()), this, SLOT(setupAdaptiveUi()), Qt::QueuedConnection);
}

void DefaultNodeAdapterBridge::modelChangedEvent()
{
    parent_->modelChangedEvent();
}

void DefaultNodeAdapterBridge::triggerGuiChanged()
{
    Q_EMIT guiChanged();
}

void DefaultNodeAdapterBridge::setupAdaptiveUi()
{
    parent_->setupAdaptiveUi();
}

void DefaultNodeAdapterBridge::enableGroup(bool enable, const std::string &group)
{
    parent_->groups_enabled[group] = enable;
}

void DefaultNodeAdapterBridge::triggerSetupAdaptiveUiRequest()
{
    Q_EMIT setupAdaptiveUiRequest();
}


/// ADAPTER
DefaultNodeAdapter::DefaultNodeAdapter(Node *adaptee, WidgetController *widget_ctrl)
    : NodeAdapter(adaptee, widget_ctrl), bridge(this), wrapper_layout_(NULL)
{
    QObject::connect(adaptee, SIGNAL(modelChanged()), &bridge, SLOT(modelChangedEvent()));
}

DefaultNodeAdapter::~DefaultNodeAdapter()
{
    clear();
}

void DefaultNodeAdapter::clear()
{
    Q_FOREACH(const boost::signals2::connection& c, connections) {
        c.disconnect();
    }

    QtHelper::clearLayout(wrapper_layout_);

    connections.clear();

    Q_FOREACH(QObject* cb, callbacks) {
        qt_helper::Call* call = dynamic_cast<qt_helper::Call*>(cb);
        if(call) {
            call->disconnect();
        }
        cb->deleteLater();
    }
    callbacks.clear();
    groups.clear();
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


class ParameterContextMenu : public ContextMenuHandler
{
public:
    ParameterContextMenu(param::Parameter *p)
        : param_(p)
    {

    }

    void doShowContextMenu(const QPoint& pt)
    {
        QWidget* w = dynamic_cast<QWidget*>(parent());
        if(!w) {
            return;
        }

        QPoint gpt = w->mapToGlobal(pt);

        QMenu menu;
        ContextMenuHandler::addHeader(menu, std::string("Parameter: ") + param_->name());

        QAction* connectable = new QAction("connectable", &menu);
        connectable->setCheckable(true);
        connectable->setChecked(param_->isInteractive());
        connectable->setIcon(QIcon(":/connector.png"));

        connectable->setIconVisibleInMenu(true);
        menu.addAction(connectable);

        QAction* selectedItem = menu.exec(gpt);
        if (selectedItem) {
            if(selectedItem == connectable) {
                param_->setInteractive(!param_->isInteractive());
            }
        }
    }

private:
    param::Parameter* param_;
};




void DefaultNodeAdapter::setupUi(QBoxLayout * outer_layout)
{
    if(!wrapper_layout_) {
        wrapper_layout_ = new QVBoxLayout;
        outer_layout->addLayout(wrapper_layout_);
    }

    setupAdaptiveUi();
}

void DefaultNodeAdapter::setupAdaptiveUi()
{
    static std::map<int, boost::function<void(DefaultNodeAdapter*, param::Parameter::Ptr)> > mapping_;
    if(mapping_.empty()) {
#define INSTALL(_TYPE_) \
    mapping_[_TYPE_().ID()] = boost::bind(static_cast<void (DefaultNodeAdapter::*)( _TYPE_* )> (&DefaultNodeAdapter::setupParameter), __1, \
    boost::bind(&boost::shared_ptr<_TYPE_>::get, boost::bind(&boost::dynamic_pointer_cast<_TYPE_, param::Parameter>, __2)))

        INSTALL(param::TriggerParameter);
        INSTALL(param::ColorParameter);
        INSTALL(param::PathParameter);
        INSTALL(param::ValueParameter);
        INSTALL(param::RangeParameter);
        INSTALL(param::IntervalParameter);
        INSTALL(param::SetParameter);
        INSTALL(param::BitSetParameter);
#undef INSTALL
    }

    clear();

    current_layout_ = wrapper_layout_;

    std::vector<param::Parameter::Ptr> params = node_->getParameters();

    GenericState::Ptr state = boost::dynamic_pointer_cast<GenericState>(node_->getState());
    if(state) {
        state->parameter_set_changed->disconnect_all_slots();
        state->parameter_set_changed->connect(boost::bind(&DefaultNodeAdapterBridge::triggerSetupAdaptiveUiRequest, &bridge));
    }

    Q_FOREACH(param::Parameter::Ptr p, params) {
        param::Parameter* parameter = p.get();

        if(!parameter->isEnabled()) {
            continue;
        }

        current_name_= parameter->name();
        current_display_name_ = current_name_;
        std::size_t separator_pos = current_name_.find_first_of('/');

        QBoxLayout* group_layout = NULL;

        if(separator_pos != std::string::npos) {
            std::string group = current_name_.substr(0, separator_pos);
            current_display_name_ = current_name_.substr(separator_pos+1);

            if(groups.find(group) != groups.end()) {
                group_layout = groups[group];
            } else {
                bool hidden = group.size() > 0 && group.at(0) == '~';

                QGroupBox* gb;
                if(hidden) {
                    gb = new QGroupBox(QString::fromStdString(group.substr(1)));
                } else {
                    gb = new QGroupBox(QString::fromStdString(group));
                }

                if(groups_enabled.find(group) != groups_enabled.end()) {
                    hidden = !groups_enabled[group];
                }

                gb->setContentsMargins(0,0,0,0);

                QVBoxLayout* gb_layout = new QVBoxLayout;
                gb->setLayout(gb_layout);
                gb->setCheckable(true);
                gb->setChecked(!hidden);
                gb_layout->setContentsMargins(0,0,0,0);

                group_layout = new QVBoxLayout;
                groups.insert(std::make_pair(group, group_layout));
                group_layout->setContentsMargins(0,0,0,0);

                QFrame* hider = new QFrame;
                hider->setLayout(group_layout);
                hider->setContentsMargins(0,0,0,0);
                gb_layout->addWidget(hider);

                wrapper_layout_->addWidget(gb);

                hider->setShown(!hidden);


                qt_helper::Call* call_trigger = new qt_helper::Call(boost::bind(&DefaultNodeAdapterBridge::enableGroup, &bridge, boost::bind(&QGroupBox::isChecked, gb), group));
                callbacks.push_back(call_trigger);
                QObject::connect(gb, SIGNAL(toggled(bool)), call_trigger, SLOT(call()));

                QObject::connect(gb, SIGNAL(toggled(bool)), hider, SLOT(setShown(bool)));
            }
        }

        current_layout_ = new QHBoxLayout;



        // connect parameter input, if available
        ConnectorIn* param_in = node_->getParameterInput(current_name_);
        if(param_in) {
            Port* port = new Port(node_->getCommandDispatcher(), param_in);
            port->setVisible(p->isInteractive());
            interactive_changed(*p).connect(boost::bind(&Port::setVisible, port, __2));

            widget_ctrl_->insertPort(current_layout_, port);
        }

        // generate UI element
        if(mapping_.find(p->ID()) != mapping_.end()) {
            mapping_[p->ID()](this, p);

        } else {
            current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet registered (value: " + type2name(p->type()) + ")").c_str()));
        }

        // connect parameter output, if available
        ConnectorOut* param_out = node_->getParameterOutput(current_name_);
        if(param_out) {
            Port* port = new Port(node_->getCommandDispatcher(), param_out);
            port->setVisible(p->isInteractive());
            interactive_changed(*p).connect(boost::bind(&Port::setVisible, port, __2));

            qt_helper::Call* call_trigger = new qt_helper::Call(boost::bind(&param::Parameter::triggerChange, p.get()));
            callbacks.push_back(call_trigger);
            QObject::connect(param_out, SIGNAL(connectionDone()), call_trigger, SLOT(call()));

            widget_ctrl_->insertPort(current_layout_, port);
        }

        // put into layout
        if(group_layout) {
            group_layout->addLayout(current_layout_);
        } else {
            wrapper_layout_->addLayout(current_layout_);
        }
    }
}

void DefaultNodeAdapter::setupParameter(param::TriggerParameter * trigger_p)
{
    QPushButton* btn = new QPushButton(trigger_p->name().c_str());

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(trigger_p)));

    boost::function<void()> cb = boost::bind(&param::TriggerParameter::trigger, trigger_p);
    qt_helper::Call* call_trigger = new qt_helper::Call(cb);
    callbacks.push_back(call_trigger);

    QObject::connect(btn, SIGNAL(clicked()), call_trigger, SLOT(call()));
}

void DefaultNodeAdapter::setupParameter(param::ColorParameter *color_p)
{
    QPushButton* btn = new QPushButton;

    btn->setStyleSheet(toColorSS(color_p->value()));

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(color_p)));

    // ui callback
    boost::function<void()> cb = boost::bind(&updateColor, color_p);
    qt_helper::Call* call = new qt_helper::Call(cb);
    callbacks.push_back(call);
    QObject::connect(btn, SIGNAL(clicked()), call, SLOT(call()));

    // model change -> ui
    boost::function<std::vector<int>()> readColor = boost::bind(&param::ColorParameter::value, color_p);
    boost::function<QString()> readSS = boost::bind(&toColorSS, boost::bind(readColor));
    boost::function<void(param::Parameter*)> up = boost::bind(&QPushButton::setStyleSheet, btn, boost::bind(readSS));

    connections.push_back(parameter_changed(*color_p).connect(up));
}

void DefaultNodeAdapter::setupParameter(param::PathParameter *path_p)
{
    QLineEdit* path = new QLineEdit(path_p->as<std::string>().c_str());
    QPushButton* open = new QPushButton("open");

    QHBoxLayout* sub = new QHBoxLayout;

    sub->addWidget(path);
    sub->addWidget(open);

    current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(path_p)));

    // ui change -> model
    boost::function<std::string(const QString&)> qstring2stdstring = boost::bind(&QString::toStdString, __1);
    boost::function<void()> cb = boost::bind(&DefaultNodeAdapter::updateParam<std::string>, this, current_name_, boost::bind(qstring2stdstring, boost::bind(&QLineEdit::text, path)));
    qt_helper::Call* call_set_path = new qt_helper::Call(cb);
    callbacks.push_back(call_set_path);

    QString filter = QString::fromStdString(path_p->filter());
    if(filter.isEmpty()) {
        filter = "All files (*.*)";
    }

    int flags = 0;
    bool is_file = path_p->isFile();
    boost::function<void()> cb_open;

    QString dir(path_p->as<std::string>().c_str());
    if(dir.startsWith("file://", Qt::CaseInsensitive)) {
        dir = dir.replace("file://", "", Qt::CaseInsensitive);
    }

    if(path_p->isOutput()) {
        if(is_file) {
            cb_open = boost::bind(&DefaultNodeAdapter::updateParamIfNotEmpty, this, current_name_,
                                  boost::bind(qstring2stdstring, boost::bind(&QFileDialog::getSaveFileName,
                                                                             (QWidget*) 0, path_p->name().c_str(), dir, filter, (QString*) 0, (QFlags<QFileDialog::Option>) flags)));
        } else {
            cb_open = boost::bind(&DefaultNodeAdapter::updateParamIfNotEmpty, this, current_name_,
                                  boost::bind(qstring2stdstring, boost::bind(&QFileDialog::getExistingDirectory,
                                                                             (QWidget*) 0, path_p->name().c_str(), dir, (QFlags<QFileDialog::Option>) flags)));
        }
    } else {
        if(is_file) {
            cb_open = boost::bind(&DefaultNodeAdapter::updateParamIfNotEmpty, this, current_name_,
                                  boost::bind(qstring2stdstring, boost::bind(&QFileDialog::getOpenFileName,
                                                                             (QWidget*) 0, path_p->name().c_str(), dir, filter, (QString*) 0, (QFlags<QFileDialog::Option>) flags)));
        } else {
            cb_open = boost::bind(&DefaultNodeAdapter::updateParamIfNotEmpty, this, current_name_,
                                  boost::bind(qstring2stdstring, boost::bind(&QFileDialog::getExistingDirectory,
                                                                             (QWidget*) 0, path_p->name().c_str(), dir, (QFlags<QFileDialog::Option>) flags)));
        }
    }
    qt_helper::Call* call_open = new qt_helper::Call(cb_open);
    callbacks.push_back(call_open);

    // model change -> ui
    boost::function<QString(const std::string&)> stdstring2qstring = boost::bind(&QString::fromStdString, __1);
    boost::function<void(const std::string&)> set = boost::bind(&QLineEdit::setText, path, boost::bind(stdstring2qstring, __1));
    connections.push_back(parameter_changed(*path_p).connect(boost::bind(&DefaultNodeAdapter::updateUi<std::string>, this, __1, set)));

    QObject::connect(path, SIGNAL(returnPressed()), call_set_path, SLOT(call()));
    QObject::connect(open, SIGNAL(clicked()), call_open, SLOT(call()));
}

void DefaultNodeAdapter::setupParameter(param::ValueParameter *value_p)
{
    if(value_p->is<std::string>()) {
        QLineEdit* txt_ = new QLineEdit;
        txt_->setText(value_p->as<std::string>().c_str());
        QPushButton* send = new QPushButton("set");

        QHBoxLayout* sub = new QHBoxLayout;

        sub->addWidget(txt_);
        sub->addWidget(send);

        current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(value_p)));

        // ui change -> model
        boost::function<std::string(const QString&)> qstring2stdstring = boost::bind(&QString::toStdString, __1);
        boost::function<void()> cb = boost::bind(&DefaultNodeAdapter::updateParam<std::string>, this, current_name_, boost::bind(qstring2stdstring, boost::bind(&QLineEdit::text, txt_)));
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);

        // model change -> ui
        boost::function<QString(const std::string&)> stdstring2qstring = boost::bind(&QString::fromStdString, __1);
        boost::function<void(const std::string&)> set = boost::bind(&QLineEdit::setText, txt_, boost::bind(stdstring2qstring, __1));
        connections.push_back(parameter_changed(*value_p).connect(boost::bind(&DefaultNodeAdapter::updateUi<std::string>, this, __1, set)));

        QObject::connect(txt_, SIGNAL(returnPressed()), call, SLOT(call()));
        QObject::connect(send, SIGNAL(clicked()), call, SLOT(call()));

    } else if(value_p->is<bool>()) {
        QCheckBox* box = new QCheckBox;
        box->setChecked(value_p->as<bool>());

        current_layout_->addLayout(QtHelper::wrap(current_display_name_, box, new ParameterContextMenu(value_p)));

        // ui change -> model
        boost::function<void()> cb = boost::bind(&DefaultNodeAdapter::updateParam<bool>, this, current_name_, boost::bind(&QCheckBox::isChecked, box));
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);

        // model change -> ui
        boost::function<void(bool)> set = boost::bind(&QCheckBox::setChecked, box, __1);
        connections.push_back(parameter_changed(*value_p).connect(boost::bind(&DefaultNodeAdapter::updateUi<bool>, this, __1, set)));

        QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

    } else if(value_p->is<double>()) {
        QDoubleSpinBox* box = new QDoubleSpinBox;
        box->setDecimals(10);
        box->setValue(value_p->as<double>());

        current_layout_->addLayout(QtHelper::wrap(current_display_name_, box, new ParameterContextMenu(value_p)));

        // ui change -> model
        boost::function<void()> cb = boost::bind(&param::ValueParameter::set<double>, value_p, boost::bind(&QDoubleSpinBox::value, box));
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);

        // model change -> ui
        connections.push_back(parameter_changed(*value_p).connect(boost::bind(&QDoubleSpinBox::setValue, box, boost::bind(&param::ValueParameter::as<double>, value_p))));

        QObject::connect(box, SIGNAL(valueChanged(double)), call, SLOT(call()));

    }  else if(value_p->is<int>()) {
        QSpinBox* box = new QSpinBox;
        box->setValue(value_p->as<int>());

        current_layout_->addLayout(QtHelper::wrap(current_display_name_, box, new ParameterContextMenu(value_p)));

        // ui change -> model
        boost::function<void()> cb = boost::bind(&param::ValueParameter::set<int>, value_p, boost::bind(&QSpinBox::value, box));
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);

        // model change -> ui
        connections.push_back(parameter_changed(*value_p).connect(boost::bind(&QSpinBox::setValue, box, boost::bind(&param::ValueParameter::as<int>, value_p))));

        QObject::connect(box, SIGNAL(valueChanged(int)), call, SLOT(call()));

    } else {
        current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet implemented (value: " + type2name(value_p->type()) + ")").c_str()));
    }
}

void DefaultNodeAdapter::setupParameter(param::RangeParameter *range_p)
{
    if(range_p->is<int>()) {
        QIntSlider* slider = QtHelper::makeIntSlider(current_layout_, current_display_name_ ,
                                                     range_p->def<int>(), range_p->min<int>(), range_p->max<int>(), range_p->step<int>(),
                                                     new ParameterContextMenu(range_p));
        slider->setIntValue(range_p->as<int>());

        // ui change -> model
        boost::function<void()> cb = boost::bind(&DefaultNodeAdapter::updateParam<int>, this, current_name_, boost::bind(&QIntSlider::intValue, slider));
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);

        // model change -> ui
        boost::function<void(int)> set = boost::bind(&QIntSlider::setIntValue, slider, __1);
        connections.push_back(parameter_changed(*range_p).connect(boost::bind(&DefaultNodeAdapter::updateUi<int>, this, __1, set)));

        QObject::connect(slider, SIGNAL(intValueChanged(int)), call, SLOT(call()));

    } else if(range_p->is<double>()) {
        QDoubleSlider* slider = QtHelper::makeDoubleSlider(current_layout_, current_display_name_ ,
                                                           range_p->def<double>(), range_p->min<double>(), range_p->max<double>(), range_p->step<double>(),
                                                           new ParameterContextMenu(range_p));
        slider->setDoubleValue(range_p->as<double>());

        // ui change -> model
        boost::function<void()> cb = boost::bind(&DefaultNodeAdapter::updateParam<double>, this, current_name_, boost::bind(&QDoubleSlider::doubleValue, slider));
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);

        // model change -> ui
        boost::function<void(double)> set = boost::bind(&QDoubleSlider::setDoubleValue, slider, __1);
        connections.push_back(parameter_changed(*range_p).connect(boost::bind(&DefaultNodeAdapter::updateUi<double>, this, __1, set)));

        QObject::connect(slider, SIGNAL(valueChanged(double)), call, SLOT(call()));

    } else {
        current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet implemented (range: " + type2name(range_p->type()) + ")").c_str()));
    }
}

void DefaultNodeAdapter::setupParameter(param::IntervalParameter *interval_p)
{
    if(interval_p->is<std::pair<int, int> >()) {
        const std::pair<int,int>& v = interval_p->as<std::pair<int,int> >();
        QxtSpanSlider* slider = QtHelper::makeSpanSlider(current_layout_, current_display_name_,
                                                         v.first, v.second, interval_p->min<int>(), interval_p->max<int>(),
                                                         new ParameterContextMenu(interval_p));

        // ui change -> model
        boost::function<int()> low = boost::bind(&QxtSpanSlider::lowerValue, slider);
        boost::function<int()> high = boost::bind(&QxtSpanSlider::upperValue, slider);
        boost::function<std::pair<int,int>()> mkpair(boost::bind(&std::make_pair<int,int>, boost::bind(low), boost::bind(high)));
        boost::function<void()> cb = boost::bind(&DefaultNodeAdapter::updateParam<std::pair<int,int> >, this, current_name_, boost::bind(mkpair));
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);

        // model change -> ui
        boost::function<void(int)> setLow = boost::bind(&QxtSpanSlider::setLowerValue, slider, __1);
        boost::function<void(int)> setHigh = boost::bind(&QxtSpanSlider::setUpperValue, slider, __1);
        boost::function<void(std::pair<int,int>)> setLowFromPair = boost::bind(setLow, boost::bind(&std::pair<int,int>::first, __1));
        boost::function<void(std::pair<int,int>)> setHighFromPair = boost::bind(setHigh, boost::bind(&std::pair<int,int>::second, __1));

        connections.push_back(parameter_changed(*interval_p).connect(boost::bind(&DefaultNodeAdapter::updateUi<std::pair<int,int> >, this, __1, setLowFromPair)));
        connections.push_back(parameter_changed(*interval_p).connect(boost::bind(&DefaultNodeAdapter::updateUi<std::pair<int,int> >, this, __1, setHighFromPair)));


        boost::function<void(int)> setMin = boost::bind(&QxtSpanSlider::setMinimum, slider, __1);
        boost::function<void(int)> setMax = boost::bind(&QxtSpanSlider::setMaximum, slider, __1);
        boost::function<void(const param::IntervalParameter*)> setMinFromParam = boost::bind(setMin, boost::bind(&param::IntervalParameter::min<int>, __1));
        boost::function<void(const param::IntervalParameter*)> setMaxFromParam = boost::bind(setMax, boost::bind(&param::IntervalParameter::max<int>, __1));

        connections.push_back(scope_changed(*interval_p).connect(boost::bind(&DefaultNodeAdapter::updateUiPtr<param::IntervalParameter>, this, __1, setMinFromParam)));
        connections.push_back(scope_changed(*interval_p).connect(boost::bind(&DefaultNodeAdapter::updateUiPtr<param::IntervalParameter>, this, __1, setMaxFromParam)));

        QObject::connect(slider, SIGNAL(lowerValueChanged(int)), call, SLOT(call()));
        QObject::connect(slider, SIGNAL(upperValueChanged(int)), call, SLOT(call()));

    } else if(interval_p->is<std::pair<double, double> >()) {
        const std::pair<double,double>& v = interval_p->as<std::pair<double,double> >();
        QxtDoubleSpanSlider* slider = QtHelper::makeDoubleSpanSlider(current_layout_, current_display_name_,
                                                                     v.first, v.second, interval_p->min<double>(), interval_p->max<double>(), interval_p->step<double>(),
                                                                     new ParameterContextMenu(interval_p));

        // ui change -> model
        boost::function<double()> low = boost::bind(&QxtDoubleSpanSlider::lowerDoubleValue, slider);
        boost::function<double()> high = boost::bind(&QxtDoubleSpanSlider::upperDoubleValue, slider);
        boost::function<std::pair<double,double>()> mkpair(boost::bind(&std::make_pair<double,double>, boost::bind(low), boost::bind(high)));
        boost::function<void()> cb = boost::bind(&DefaultNodeAdapter::updateParam<std::pair<double,double> >, this, current_name_, boost::bind(mkpair));
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);

        // model change -> ui
        boost::function<void(double, double)> setSpan = boost::bind(&QxtDoubleSpanSlider::setSpan, slider, __1, __2);
        boost::function<void(std::pair<double,double>)> setSpanFromPair = boost::bind(setSpan,
                                                                                      boost::bind(&std::pair<double,double>::first, __1),
                                                                                      boost::bind(&std::pair<double,double>::second, __1));
        connections.push_back(parameter_changed(*interval_p).connect(boost::bind(&DefaultNodeAdapter::updateUi<std::pair<double,double> >,
                                                                                 this, __1, setSpanFromPair)));


        boost::function<void(double,double)> setInterval = boost::bind(&QxtDoubleSpanSlider::setDoubleRange, slider, __1, __2);
        boost::function<void(const param::IntervalParameter*)> setFromParam = boost::bind(setInterval,
                                                                                          boost::bind(&param::IntervalParameter::min<double>, __1),
                                                                                          boost::bind(&param::IntervalParameter::max<double>, __1));
        connections.push_back(scope_changed(*interval_p).connect(boost::bind(&DefaultNodeAdapter::updateUiPtr<param::IntervalParameter>, this, __1, setFromParam)));

        //        boost::function<void(double)> setMin = boost::bind(&QxtDoubleSpanSlider::setDoubleMinimum, slider, __1);
        //        boost::function<void(double)> setMax = boost::bind(&QxtDoubleSpanSlider::setDoubleMaximum, slider, __1);
        //        boost::function<void(const param::IntervalParameter*)> setMinFromParam = boost::bind(setMin, boost::bind(&param::IntervalParameter::min<double>, __1));
        //        boost::function<void(const param::IntervalParameter*)> setMaxFromParam = boost::bind(setMax, boost::bind(&param::IntervalParameter::max<double>, __1));

        //        connections.push_back(scope_changed(*interval_p).connect(boost::bind(&DefaultNodeAdapter::updateUiPtr<param::IntervalParameter>, this, __1, setMinFromParam)));
        //        connections.push_back(scope_changed(*interval_p).connect(boost::bind(&DefaultNodeAdapter::updateUiPtr<param::IntervalParameter>, this, __1, setMaxFromParam)));

        QObject::connect(slider, SIGNAL(lowerValueChanged(int)), call, SLOT(call()));
        QObject::connect(slider, SIGNAL(upperValueChanged(int)), call, SLOT(call()));


    } else {
        current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet implemented (inverval: " + type2name(interval_p->type()) + ")").c_str()));
    }
}

void DefaultNodeAdapter::setupParameter(param::SetParameter *set_p)
{
    QComboBox* combo = new QComboBox;

    updateUiSetScope(set_p, combo);
    current_layout_->addLayout(QtHelper::wrap(current_display_name_, combo, new ParameterContextMenu(set_p)));

    // ui change -> model
    boost::function<std::string(const QString&)> qstring2stdstring = boost::bind(&QString::toStdString, __1);
    boost::function<void()> cb = boost::bind(&DefaultNodeAdapter::updateParamSet, this, current_name_, boost::bind(qstring2stdstring, boost::bind(&QComboBox::currentText, combo)));
    qt_helper::Call* call = new qt_helper::Call(cb);
    callbacks.push_back(call);

    // model change -> ui
    boost::function<QString(const std::string&)> stdstring2qstring = boost::bind(&QString::fromStdString, __1);
    boost::function<int(const QString&)> txt2idx = boost::bind(&QComboBox::findData, combo, __1, Qt::DisplayRole, static_cast<Qt::MatchFlags>(Qt::MatchExactly|Qt::MatchCaseSensitive));
    boost::function<void(const QString&)> select = boost::bind(&QComboBox::setCurrentIndex, combo, boost::bind(txt2idx, __1));
    boost::function<void(const std::string&)> set = boost::bind(select, boost::bind(stdstring2qstring, __1));
    connections.push_back(parameter_changed(*set_p).connect(boost::bind(&DefaultNodeAdapter::updateUiSet, this, __1, set)));

    connections.push_back(scope_changed(*set_p).connect(boost::bind(&DefaultNodeAdapter::updateUiSetScope, this, set_p, combo)));

    QObject::connect(combo, SIGNAL(currentIndexChanged(QString)), call, SLOT(call()));
}

void DefaultNodeAdapter::setupParameter(param::BitSetParameter *bitset_p)
{
    QGroupBox* group = new QGroupBox(current_name_.c_str());
    QVBoxLayout* l = new QVBoxLayout;
    group->setLayout(l);
    for(int i = 0; i < bitset_p->noParameters(); ++i) {
        std::string str = bitset_p->getName(i);
        QCheckBox* item = new QCheckBox(QString::fromStdString(str));
        l->addWidget(item);
        if(bitset_p->isSet(str)) {
            item->setChecked(true);
        }

        // ui change -> model
        boost::function<bool()> isChecked = boost::bind(&QCheckBox::isChecked, item);
        boost::function<void()> cb = boost::bind(&param::BitSetParameter::setBitTo, bitset_p, str, boost::bind(isChecked), false);
        qt_helper::Call* call = new qt_helper::Call(cb);
        callbacks.push_back(call);
        QObject::connect(item, SIGNAL(toggled(bool)), call, SLOT(call()));

        // model change -> ui
        boost::function<bool()> isSet = boost::bind(&param::BitSetParameter::isSet, bitset_p, str);
        connections.push_back(parameter_changed(*bitset_p).connect(boost::bind(&QCheckBox::setChecked, item, boost::bind(isSet))));
    }
    current_layout_->addWidget(group);
}

template <typename T>
void DefaultNodeAdapter::updateParam(const std::string& name, T value)
{
    param::Parameter* p = node_->getParameter(name).get();
    p->set<T>(value);
    guiChanged();
}

void DefaultNodeAdapter::updateParamIfNotEmpty(const std::string& name, const std::string& value)
{
    if(value.empty()) {
        return;
    }

    updateParam(name, value);
}


void DefaultNodeAdapter::updateParamSet(const std::string& name, const std::string& value)
{
    param::SetParameter::Ptr set_p = node_->getParameter<param::SetParameter>(name);
    if(set_p) {
        set_p->setByName(value);
        guiChanged();
    }
}

void DefaultNodeAdapter::updateParamBitSet(const std::string& name, const QListView* list)
{
    param::BitSetParameter::Ptr bitset_p = node_->getParameter<param::BitSetParameter>(name);
    if(bitset_p) {
        std::vector<std::string> selected;
        Q_FOREACH(const QModelIndex& idx, list->selectionModel()->selectedIndexes()) {
            selected.push_back(idx.data().toString().toStdString());
        }
        bitset_p->setBits(selected, true);
        guiChanged();
    }
}

template <typename T>
void DefaultNodeAdapter::updateUi(const param::Parameter* p, boost::function<void(T)> setter)
{
    /// TODO: execute ONLY in UI thread
    setter(node_->param<T>(p->name()));
}

template <typename T>
void DefaultNodeAdapter::updateUiPtr(const param::Parameter* p, boost::function<void(const T*)> setter)
{
    /// TODO: execute ONLY in UI thread
    setter(dynamic_cast<const T*>(p));
}

void DefaultNodeAdapter::updateUiSet(const param::Parameter *p, boost::function<void (const std::string &)> setter)
{
    /// TODO: execute ONLY in UI thread
    const param::SetParameter* set_p = dynamic_cast<const param::SetParameter*> (p);
    if(set_p) {
        setter(set_p->getName());
    }
}

void DefaultNodeAdapter::updateUiSetScope(const param::SetParameter *set_p, QComboBox *combo)
{
    /// TODO: execute ONLY in UI thread
    int current = 0;
    combo->clear();
    std::string selected;
    try {
        selected = set_p->getName();
    } catch(const std::exception& e) {
        selected = "";
    }

    for(int i = 0; i < set_p->noParameters(); ++i) {
        std::string str = set_p->getName(i);
        combo->addItem(QString::fromStdString(str));

        if(str == selected) {
            current = i;
        }
    }
    combo->setCurrentIndex(current);
}

void DefaultNodeAdapter::updateUiBitSet(const param::Parameter *p, const QListView *list)
{
    /// TODO: execute ONLY in UI thread
    const param::BitSetParameter* bitset_p = dynamic_cast<const param::BitSetParameter*> (p);
    if(bitset_p) {
        for(int i = 0; i < bitset_p->noParameters(); ++i) {
            QModelIndex idx = list->model()->index(i,0);
            std::string str = bitset_p->getName(i);
            list->selectionModel()->select(idx, bitset_p->isSet(str) ? QItemSelectionModel::Select : QItemSelectionModel::Deselect);
        }
    }
}

void DefaultNodeAdapter::stop()
{
    NodeAdapter::stop();
    bridge.disconnect();
}

void DefaultNodeAdapter::guiChanged()
{
    NodeAdapter::guiChanged();
    bridge.triggerGuiChanged();
}
