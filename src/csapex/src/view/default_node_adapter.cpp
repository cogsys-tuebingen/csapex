/// HEADER
#include <csapex/view/default_node_adapter.h>

/// COMPONENT
#include <csapex/utility/qt_helper.hpp>
#include <csapex/utility/q_signal_relay.h>
#include <csapex/model/node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/view/port.h>
#include <csapex/view/widget_controller.h>
#include <csapex/model/node_worker.h>
#include <csapex/view/parameter_context_menu.h>

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
#include <QListWidget>
#include <QApplication>

using namespace csapex;
using namespace boost::lambda;
namespace bll = boost::lambda;

boost::arg<1> __1;
boost::arg<2> __2;

namespace {

void assertGuiThread()
{
    assert(QThread::currentThread() == QApplication::instance()->thread());
}

void assertNotGuiThread()
{
    assert(QThread::currentThread() != QApplication::instance()->thread());
}
}

/// BRIDGE
DefaultNodeAdapterBridge::DefaultNodeAdapterBridge(DefaultNodeAdapter *parent)
    : parent_(parent)
{
    qRegisterMetaType < Function > ("Function");

    assertGuiThread();
    assert(thread() == QApplication::instance()->thread());

    QObject::connect(this, SIGNAL(setupAdaptiveUiRequest()), this, SLOT(setupAdaptiveUi()), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(modelCallback(Function)), this, SLOT(executeModelCallback(Function)));
}

void DefaultNodeAdapterBridge::connectInGuiThread(boost::signals2::signal<void (param::Parameter *)> &signal,
                                                  boost::function<void ()> cb)
{
    // cb should be executed in the gui thread
    connections.push_back(signal.connect(boost::bind(&DefaultNodeAdapterBridge::modelCallback, this, cb)));
}

void DefaultNodeAdapterBridge::disconnect()
{
    Q_FOREACH(const boost::signals2::connection& c, connections) {
        c.disconnect();
    }

    connections.clear();
}

void DefaultNodeAdapterBridge::executeModelCallback(Function cb)
{
    assertGuiThread();
    cb();
}

void DefaultNodeAdapterBridge::nodeModelChangedEvent()
{
    parent_->nodeModelChangedEvent();
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
DefaultNodeAdapter::DefaultNodeAdapter(NodeWorker *adaptee, WidgetController *widget_ctrl)
    : NodeAdapter(adaptee, widget_ctrl), bridge(this), wrapper_layout_(NULL)
{
    QObject::connect(adaptee, SIGNAL(nodeModelChanged()), &bridge, SLOT(nodeModelChangedEvent()));
}

DefaultNodeAdapter::~DefaultNodeAdapter()
{
    clear();
}

void DefaultNodeAdapter::clear()
{
    bridge.disconnect();

    QtHelper::clearLayout(wrapper_layout_);

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

// COLOR ////////////////////
void ui_updateColorParameter(param::ColorParameter* color_p, QPushButton* btn)
{
    assertGuiThread();
    btn->setStyleSheet(toColorSS(color_p->value()));
}

void model_updateColorParameter(param::ColorParameter *p)
{
    assertNotGuiThread();
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

// PATH ////////////////////
void ui_updatePathParameter(param::PathParameter* path_p, QLineEdit* path)
{
    assertGuiThread();
    path->setText(QString::fromStdString(path_p->as<std::string>()));
}

void model_updatePathParameter(param::PathParameter* path_p, QLineEdit* path)
{
    assertNotGuiThread();
    path_p->set<std::string>(path->text().toStdString());
    // gui changed
}

void ui_updatePathParameterDialog(param::PathParameter* path_p)
{
    assertGuiThread();
    QString filter = QString::fromStdString(path_p->filter());
    if(filter.isEmpty()) {
        filter = "All files (*.*)";
    }

    int flags = 0;
    bool is_file = path_p->isFile();

    QString dir(path_p->as<std::string>().c_str());
    if(dir.startsWith("file://", Qt::CaseInsensitive)) {
        dir = dir.replace("file://", "", Qt::CaseInsensitive);
    }

    QString path;
    if(path_p->isOutput()) {
        if(is_file) {
            path = QFileDialog::getSaveFileName((QWidget*) 0, path_p->name().c_str(), dir, filter, (QString*) 0, (QFlags<QFileDialog::Option>) flags);
        } else {
            path = QFileDialog::getExistingDirectory((QWidget*) 0, path_p->name().c_str(), dir, (QFlags<QFileDialog::Option>) flags);
        }
    } else {
        if(is_file) {
            path = QFileDialog::getOpenFileName((QWidget*) 0, path_p->name().c_str(), dir, filter, (QString*) 0, (QFlags<QFileDialog::Option>) flags);
        } else {
            path = QFileDialog::getExistingDirectory((QWidget*) 0, path_p->name().c_str(), dir, (QFlags<QFileDialog::Option>) flags);
        }
    }

    if(!path.isEmpty()) {
        path_p->set(path.toStdString());
        //        cb();
    }
}

// VALUE ////////////////////
void ui_updateStringValueParameter(param::ValueParameter* value_p, QLineEdit* txt)
{
    assertGuiThread();
    txt->setText(QString::fromStdString(value_p->as<std::string>()));
}
void model_updateStringValueParameter(param::ValueParameter* value_p, QLineEdit* txt)
{
    assertNotGuiThread();
    value_p->set<std::string>(txt->text().toStdString());
}

void ui_updateBoolValueParameter(param::ValueParameter* value_p, QCheckBox* cb)
{
    assertGuiThread();
    cb->setChecked(value_p->as<bool>());
}
void model_updateBoolValueParameter(param::ValueParameter* value_p, QCheckBox* cb)
{
    assertNotGuiThread();
    value_p->set<bool>(cb->isChecked());
}

template <typename T, typename Widget>
void ui_updateValueParameter(param::ValueParameter* value_p, Widget* box)
{
    assertGuiThread();
    box->setValue(value_p->as<T>());
}
template <typename T, typename Widget>
void model_updateValueParameter(param::ValueParameter* value_p, Widget* box)
{
    assertNotGuiThread();
    value_p->set<T>(box->value());
}

// RANGE ////////////////////
void ui_updateIntRangeParameter(param::RangeParameter* range_p, QIntSlider* slider)
{
    assertGuiThread();
    slider->setIntValue(range_p->as<int>());
}
void model_updateIntRangeParameter(param::RangeParameter* range_p, QIntSlider* slider)
{
    assertNotGuiThread();
    range_p->set<int>(slider->intValue());
}

void ui_updateIntRangeParameterScope(param::RangeParameter* range_p, QIntSlider* slider)
{
    assertGuiThread();
    slider->setIntRange(range_p->min<int>(), range_p->max<int>());
}

void ui_updateDoubleRangeParameter(param::RangeParameter* range_p, QDoubleSlider* slider)
{
    assertGuiThread();
    slider->setDoubleValue(range_p->as<double>());
}
void model_updateDoubleRangeParameter(param::RangeParameter* range_p, QDoubleSlider* slider)
{
    assertNotGuiThread();
    range_p->set<double>(slider->doubleValue());
}

void ui_updateDoubleRangeParameterScope(param::RangeParameter* range_p, QDoubleSlider* slider)
{
    assertGuiThread();
    slider->setDoubleRange(range_p->min<double>(), range_p->max<double>());
}

// INTERVAL ////////////////////
template <typename T, typename Widget>
void ui_updateIntervalParameter(param::IntervalParameter* interval_p, Widget* slider)
{
    assertGuiThread();
    slider->setSpan(interval_p->lower<T>(), interval_p->upper<T>());
}
template <typename T, typename Widget>
void model_updateIntervalParameter(param::IntervalParameter* interval_p, Widget* slider)
{
    assertNotGuiThread();
    interval_p->set(std::make_pair(slider->lowerValue(), slider->upperValue()));
}

template <typename T, typename Widget>
void ui_updateIntervalParameterScope(param::IntervalParameter* interval_p, Widget* slider)
{
    assertGuiThread();
    slider->setRange(interval_p->min<T>(), interval_p->max<T>());
}

// SET ////////////////////
void ui_updateSetParameter(param::SetParameter* set_p, QComboBox* combo)
{
    assertGuiThread();
    int index = combo->findData(QString::fromStdString(set_p->getText()));
    if(index >= 0) {
        combo->setCurrentIndex(index);
    }
}
void model_updateSetParameter(param::SetParameter* set_p, QComboBox* combo)
{
    if(!combo->currentText().isEmpty()) {
        std::cout << "set set: " << combo->currentText().toStdString() << std::endl;
        assertNotGuiThread();
        set_p->setByName(combo->currentText().toStdString());
    }
}

void ui_updateSetParameterScope(param::SetParameter* set_p, QComboBox* combo)
{
    assertGuiThread();

    int current = 0;
    combo->clear();
    std::string selected;
    try {
        selected = set_p->getText();
    } catch(const std::exception& e) {
        selected = "";
    }

    for(int i = 0; i < set_p->noParameters(); ++i) {
        std::string str = set_p->getText(i);
        combo->addItem(QString::fromStdString(str));

        if(str == selected) {
            current = i;
        }
    }
    combo->setCurrentIndex(current);
}

// BITSET ////////////////////
void ui_updateBitSetParameter(param::BitSetParameter* bitset_p, QCheckBox* item, const std::string& str)
{
    assertGuiThread();
    item->setChecked(bitset_p->isSet(str));
}
void model_updateBitSetParameter(param::BitSetParameter* bitset_p, QCheckBox* item, const std::string& str)
{
    assertNotGuiThread();
    bitset_p->setBitTo(str, item->isChecked());
}


}


void DefaultNodeAdapter::setupUi(QBoxLayout * outer_layout)
{
    if(!wrapper_layout_) {
        wrapper_layout_ = new QVBoxLayout;
        outer_layout->addLayout(wrapper_layout_);
    }

    setupAdaptiveUi();
}

namespace {
void setTooltip(QLayout* l, const QString& tooltip)
{
    for(int i = 0; i < l->count(); ++i) {
        QLayoutItem* o = l->itemAt(i);
        QWidgetItem* w = dynamic_cast<QWidgetItem*>(o);
        if(w) {
            w->widget()->setToolTip(tooltip);
        } else {
            QLayout* l = dynamic_cast<QLayout*>(o);
            if(l) {
                setTooltip(l, tooltip);
            }
        }
    }
}
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

    std::vector<param::Parameter::Ptr> params = node_->getNode()->getParameters();

    GenericState::Ptr state = boost::dynamic_pointer_cast<GenericState>(node_->getNode()->getParameterState());
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


        CommandDispatcher* cmd_dispatcher = widget_ctrl_->getCommandDispatcher();


        // connect parameter input, if available
        Input* param_in = node_->getParameterInput(current_name_);
        if(param_in) {
            Port* port = new Port(cmd_dispatcher, param_in, false);
            port->setVisible(p->isInteractive());
            p->interactive_changed.connect(boost::bind(&Port::setVisible, port, __2));

            widget_ctrl_->insertPort(current_layout_, port);
        }

        // generate UI element
        if(mapping_.find(p->ID()) != mapping_.end()) {
            mapping_[p->ID()](this, p);

        } else {
            current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet registered (value: " + type2name(p->type()) + ")").c_str()));
        }

        // connect parameter output, if available
        Output* param_out = node_->getParameterOutput(current_name_);
        if(param_out) {
            Port* port = new Port(cmd_dispatcher, param_out, false);
            port->setVisible(p->isInteractive());
            p->interactive_changed.connect(boost::bind(&Port::setVisible, port, __2));

            qt_helper::Call* call_trigger = new qt_helper::Call(boost::bind(&param::Parameter::triggerChange, p.get()));
            callbacks.push_back(call_trigger);
            QObject::connect(param_out, SIGNAL(connectionDone(Connectable*)), call_trigger, SLOT(call()));

            widget_ctrl_->insertPort(current_layout_, port);
        }

        QString tooltip = QString::fromStdString(p->description().toString());
        if(!tooltip.isEmpty()){
            setTooltip(current_layout_, tooltip);
        }

        // put into layout
        if(group_layout) {
            group_layout->addLayout(current_layout_);
        } else {
            wrapper_layout_->addLayout(current_layout_);
        }
    }
}

qt_helper::Call * DefaultNodeAdapter::makeModelCall(boost::function<void()> cb)
{
    qt_helper::Call* call = new qt_helper::Call(cb);
    callbacks.push_back(call);
    call->moveToThread(node_->thread());
    return call;
}

qt_helper::Call * DefaultNodeAdapter::makeUiCall(boost::function<void()> cb)
{
    qt_helper::Call* call = new qt_helper::Call(cb);
    callbacks.push_back(call);
    return call;
}

void DefaultNodeAdapter::setupParameter(param::TriggerParameter * trigger_p)
{
    QPushButton* btn = new QPushButton(trigger_p->name().c_str());

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(trigger_p)));

    qt_helper::Call* call_trigger = makeModelCall(boost::bind(&param::TriggerParameter::trigger, trigger_p));
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
    qt_helper::Call* call = makeModelCall(boost::bind(&model_updateColorParameter, color_p));
    QObject::connect(btn, SIGNAL(clicked()), call, SLOT(call()));

    // model change -> ui
    bridge.connectInGuiThread(color_p->parameter_changed, boost::bind(&ui_updateColorParameter, color_p, btn));
}

void DefaultNodeAdapter::setupParameter(param::PathParameter *path_p)
{
    QLineEdit* path = new QLineEdit(path_p->as<std::string>().c_str());
    QPushButton* select = new QPushButton("select");

    QHBoxLayout* sub = new QHBoxLayout;

    sub->addWidget(path);
    sub->addWidget(select);

    current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(path_p)));

    // ui change -> model
    qt_helper::Call* call_set_path = makeModelCall(boost::bind(&model_updatePathParameter, path_p, path));
    QObject::connect(path, SIGNAL(returnPressed()), call_set_path, SLOT(call()));

    qt_helper::Call* call_select = makeUiCall(boost::bind(&ui_updatePathParameterDialog, path_p));
    QObject::connect(select, SIGNAL(clicked()), call_select, SLOT(call()));

    // model change -> ui
    bridge.connectInGuiThread(path_p->parameter_changed, boost::bind(&ui_updatePathParameter, path_p, path));
}

void DefaultNodeAdapter::setupParameter(param::ValueParameter *value_p)
{
    if(value_p->is<std::string>()) {
        QLineEdit* txt = new QLineEdit;
        txt->setText(value_p->as<std::string>().c_str());
        QPushButton* send = new QPushButton("set");

        QHBoxLayout* sub = new QHBoxLayout;

        sub->addWidget(txt);
        sub->addWidget(send);

        current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(value_p)));

        // ui change -> model
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateStringValueParameter, value_p, txt));
        QObject::connect(txt, SIGNAL(returnPressed()), call, SLOT(call()));
        QObject::connect(send, SIGNAL(clicked()), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(value_p->parameter_changed, boost::bind(&ui_updateStringValueParameter, value_p, txt));

    } else if(value_p->is<bool>()) {
        QCheckBox* box = new QCheckBox;
        box->setChecked(value_p->as<bool>());

        current_layout_->addLayout(QtHelper::wrap(current_display_name_, box, new ParameterContextMenu(value_p)));

        // ui change -> model
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateBoolValueParameter, value_p, box));
        QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(value_p->parameter_changed, boost::bind(&ui_updateBoolValueParameter, value_p, box));


    } else if(value_p->is<double>()) {
        QDoubleSpinBox* box = new QDoubleSpinBox;
        box->setDecimals(10);
        box->setValue(value_p->as<double>());

        current_layout_->addLayout(QtHelper::wrap(current_display_name_, box, new ParameterContextMenu(value_p)));

        // ui change -> model
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateValueParameter<double, QDoubleSpinBox>, value_p, box));
        QObject::connect(box, SIGNAL(valueChanged(double)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(value_p->parameter_changed, boost::bind(&ui_updateValueParameter<double, QDoubleSpinBox>, value_p, box));

    }  else if(value_p->is<int>()) {
        QSpinBox* box = new QSpinBox;
        box->setValue(value_p->as<int>());

        current_layout_->addLayout(QtHelper::wrap(current_display_name_, box, new ParameterContextMenu(value_p)));

        // ui change -> model
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateValueParameter<int, QSpinBox>, value_p, box));
        QObject::connect(box, SIGNAL(valueChanged(int)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(value_p->parameter_changed, boost::bind(&ui_updateValueParameter<int, QSpinBox>, value_p, box));

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
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateIntRangeParameter, range_p, slider));
        QObject::connect(slider, SIGNAL(intValueChanged(int)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(range_p->parameter_changed, boost::bind(&ui_updateIntRangeParameter, range_p, slider));

        // parameter scope changed -> update slider interval
        bridge.connectInGuiThread(range_p->scope_changed, boost::bind(&ui_updateIntRangeParameterScope, range_p, slider));

    } else if(range_p->is<double>()) {
        QDoubleSlider* slider = QtHelper::makeDoubleSlider(current_layout_, current_display_name_ ,
                                                           range_p->def<double>(), range_p->min<double>(), range_p->max<double>(), range_p->step<double>(),
                                                           new ParameterContextMenu(range_p));
        slider->setDoubleValue(range_p->as<double>());

        // ui change -> model
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateDoubleRangeParameter, range_p, slider));
        QObject::connect(slider, SIGNAL(doubleValueChanged(double)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(range_p->parameter_changed, boost::bind(&ui_updateDoubleRangeParameter, range_p, slider));

        // parameter scope changed -> update slider interval
        bridge.connectInGuiThread(range_p->scope_changed, boost::bind(&ui_updateDoubleRangeParameterScope, range_p, slider));

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
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateIntervalParameter<int, QxtSpanSlider>, interval_p, slider));
        QObject::connect(slider, SIGNAL(lowerValueChanged(int)), call, SLOT(call()));
        QObject::connect(slider, SIGNAL(upperValueChanged(int)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(interval_p->parameter_changed, boost::bind(&ui_updateIntervalParameter<int, QxtSpanSlider>, interval_p, slider));

        // parameter scope changed -> update slider interval
        bridge.connectInGuiThread(interval_p->scope_changed, boost::bind(&ui_updateIntervalParameterScope<int, QxtSpanSlider>, interval_p, slider));

    } else if(interval_p->is<std::pair<double, double> >()) {
        const std::pair<double,double>& v = interval_p->as<std::pair<double,double> >();
        QxtDoubleSpanSlider* slider = QtHelper::makeDoubleSpanSlider(current_layout_, current_display_name_,
                                                                     v.first, v.second, interval_p->min<double>(), interval_p->max<double>(), interval_p->step<double>(),
                                                                     new ParameterContextMenu(interval_p));

        // ui change -> model
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateIntervalParameter<double, QxtDoubleSpanSlider>, interval_p, slider));
        QObject::connect(slider, SIGNAL(lowerValueChanged(int)), call, SLOT(call()));
        QObject::connect(slider, SIGNAL(upperValueChanged(int)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(interval_p->parameter_changed, boost::bind(&ui_updateIntervalParameter<double, QxtDoubleSpanSlider>, interval_p, slider));

        // parameter scope changed -> update slider interval
        bridge.connectInGuiThread(interval_p->scope_changed, boost::bind(&ui_updateIntervalParameterScope<double, QxtDoubleSpanSlider>, interval_p, slider));

    } else {
        current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet implemented (inverval: " + type2name(interval_p->type()) + ")").c_str()));
    }
}

void DefaultNodeAdapter::setupParameter(param::SetParameter *set_p)
{
    QComboBox* combo = new QComboBox;

    ui_updateSetParameterScope(set_p, combo);
    current_layout_->addLayout(QtHelper::wrap(current_display_name_, combo, new ParameterContextMenu(set_p)));

    // ui change -> model
    qt_helper::Call* call = makeModelCall(boost::bind(&model_updateSetParameter, set_p, combo));
    QObject::connect(combo, SIGNAL(currentIndexChanged(QString)), call, SLOT(call()));

    // model change -> ui
    bridge.connectInGuiThread(set_p->parameter_changed, boost::bind(&ui_updateSetParameter, set_p, combo));

    // parameter scope changed -> update slider interval
    bridge.connectInGuiThread(set_p->scope_changed, boost::bind(&ui_updateSetParameterScope, set_p, combo));

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
        qt_helper::Call* call = makeModelCall(boost::bind(&model_updateBitSetParameter, bitset_p, item, str));
        QObject::connect(item, SIGNAL(toggled(bool)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(bitset_p->parameter_changed, boost::bind(&ui_updateBitSetParameter, bitset_p, item, str));
    }

    current_layout_->addWidget(group);
}


void DefaultNodeAdapter::stop()
{
    NodeAdapter::stop();
    bridge.disconnect();
}
