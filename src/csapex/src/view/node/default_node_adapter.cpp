/// HEADER
#include <csapex/view/node/default_node_adapter.h>

/// COMPONENT
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/utility/qsignal_relay.h>
#include <csapex/model/node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/view/widgets/port.h>
#include <csapex/model/node_handle.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph_facade.h>
#include <csapex/view/widgets/doublespanslider.h>
#include <csapex/view/node/box.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/utility/timer.h>

/// PROJECT
#include <csapex/param/range_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/set_parameter.h>
#include <csapex/param/bitset_parameter.h>
#include <csapex/param/path_parameter.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/param/color_parameter.h>
#include <csapex/param/angle_parameter.h>
#include <csapex/param/output_progress_parameter.h>

#include <csapex/view/param/range_param_adapter.h>
#include <csapex/view/param/value_param_adapter.h>


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
#include <QProgressBar>
#include <QDial>
#include <functional>
#include <QLabel>
#include <qxt5/qxtspanslider.h>
#include <QPointer>

using namespace csapex;
using namespace boost::lambda;
namespace bll = boost::lambda;

//boost::arg<1> __1;
//boost::arg<2> __2;

namespace {


/// UI HANDLES


QxtSpanSlider* makeSpanSlider(QBoxLayout* layout, const std::string& name, int lower, int upper, int min, int max,
                              param::IntervalParameterPtr interval_param,
                              csapex::ContextMenuHandler *context_handler)
{
    apex_assert_hard(min<=max);

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QxtSpanSlider* slider = new QxtSpanSlider(Qt::Horizontal);
    slider->setMinimum(min);
    slider->setMaximum(max);
    slider->setUpperValue(upper);
    slider->setLowerValue(lower);

    QWrapper::QSpinBoxExt* displayLower = new QWrapper::QSpinBoxExt;
    displayLower->setMinimum(min);
    displayLower->setMaximum(max);
    displayLower->setValue(lower);

    QWrapper::QSpinBoxExt* displayUpper = new QWrapper::QSpinBoxExt;
    displayUpper->setMinimum(min);
    displayUpper->setMaximum(max);
    displayUpper->setValue(upper);

    QLabel* label = new QLabel(name.c_str());
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));
    }

    internal_layout->addWidget(label);
    internal_layout->addWidget(displayLower);
    internal_layout->addWidget(slider);
    internal_layout->addWidget(displayUpper);

    layout->addLayout(internal_layout);

    for(int i = 0; i < internal_layout->count(); ++i) {
        QWidget* child = internal_layout->itemAt(i)->widget();
        child->setProperty("parameter", QVariant::fromValue(static_cast<void*>(static_cast<csapex::param::Parameter*>(interval_param.get()))));
    }

    QObject::connect(slider,        SIGNAL(rangeChanged(int,int)),  displayLower,   SLOT(setRange(int,int)));
    QObject::connect(slider,        SIGNAL(rangeChanged(int,int)),  displayUpper,   SLOT(setRange(int,int)));
    QObject::connect(slider,        SIGNAL(lowerValueChanged(int)), displayLower,   SLOT(setValue(int)));
    QObject::connect(slider,        SIGNAL(upperValueChanged(int)), displayUpper,   SLOT(setValue(int)));
    QObject::connect(displayLower,  SIGNAL(valueChanged(int)),      slider,         SLOT(setLowerValue(int)));
    QObject::connect(displayUpper,  SIGNAL(valueChanged(int)),      slider,         SLOT(setUpperValue(int)));

    return slider;
}


DoubleSpanSlider* makeDoubleSpanSlider(QBoxLayout *layout, const std::string &name, double lower, double upper, double min, double max, double step_size,
                                          csapex::ContextMenuHandler *context_handler)
{
    apex_assert_hard(min<=max);

    QHBoxLayout* internal_layout = new QHBoxLayout;

    DoubleSpanSlider* slider = new DoubleSpanSlider(Qt::Horizontal, step_size);
    slider->setDoubleRange(min, max);
    slider->setSpan(lower, upper);

    QWrapper::QDoubleSpinBoxExt* displayLower = new QWrapper::QDoubleSpinBoxExt;
    displayLower->setRange(min, max);
    displayLower->setValue(lower);

    QWrapper::QDoubleSpinBoxExt* displayUpper = new QWrapper::QDoubleSpinBoxExt;
    displayUpper->setRange(min, max);
    displayUpper->setValue(upper);

    QLabel* label = new QLabel(name.c_str());
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));
    }

    internal_layout->addWidget(label);
    internal_layout->addWidget(displayLower);
    internal_layout->addWidget(slider);
    internal_layout->addWidget(displayUpper);

    layout->addLayout(internal_layout);

    QObject::connect(slider,        SIGNAL(rangeChanged(double,double)),  displayUpper,   SLOT(setRange(double,double)));
    QObject::connect(slider,        SIGNAL(rangeChanged(double,double)),  displayLower,   SLOT(setRange(double,double)));
    QObject::connect(slider,        SIGNAL(lowerValueChanged(double)), displayLower,   SLOT(setValue(double)));
    QObject::connect(slider,        SIGNAL(upperValueChanged(double)), displayUpper,   SLOT(setValue(double)));
    QObject::connect(displayLower,  SIGNAL(valueChanged(double)),      slider,         SLOT(setLowerDoubleValue(double)));
    QObject::connect(displayUpper,  SIGNAL(valueChanged(double)),      slider,         SLOT(setUpperDoubleValue(double)));

    return slider;
}

/// THREADING
void assertGuiThread()
{
    apex_assert_hard(QThread::currentThread() == QApplication::instance()->thread());
}

void assertNotGuiThread()
{
    apex_assert_hard(QThread::currentThread() != QApplication::instance()->thread());
}
}

/// BRIDGE
DefaultNodeAdapterBridge::DefaultNodeAdapterBridge(DefaultNodeAdapter *parent)
    : parent_(parent)
{
    qRegisterMetaType < Function > ("Function");

    assertGuiThread();
    apex_assert_hard(thread() == QApplication::instance()->thread());

    QObject::connect(this, SIGNAL(setupAdaptiveUiRequest()), this, SLOT(setupAdaptiveUi()), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(modelCallback(Function)), this, SLOT(executeModelCallback(Function)));
}

DefaultNodeAdapterBridge::~DefaultNodeAdapterBridge()
{
    disconnect();
}

void DefaultNodeAdapterBridge::connectInGuiThread(csapex::slim_signal::Signal<void (csapex::param::Parameter *)> &signal,
                                                  std::function<void ()> cb)
{
    // cb should be executed in the gui thread
    connections.push_back(signal.connect(std::bind(&DefaultNodeAdapterBridge::modelCallback, this, cb)));
}

void DefaultNodeAdapterBridge::disconnect()
{
    for(const csapex::slim_signal::Connection& c : connections) {
        c.disconnect();
    }

    connections.clear();
}

void DefaultNodeAdapterBridge::executeModelCallback(Function cb)
{
    assertGuiThread();
    cb();
}

void DefaultNodeAdapterBridge::setupAdaptiveUi()
{
//    Timer timer("setup");
    NodeHandlePtr node_handle = parent_->node_.lock();
    if(!node_handle) {
        return;
    }
    parent_->setupAdaptiveUi();
//    timer.finish();
//    for(auto pair : timer.entries()) {
//        std::cerr << pair.second << " ms" << std::endl;
//    }

}

void DefaultNodeAdapterBridge::enableGroup(bool enable, const std::string &group)
{
    parent_->groups_enabled[group] = enable;
    parent_->groupsboxes[group]->setProperty("hidden", !enable);
    QWidget* w = parent_->wrapper_layout_->parentWidget();
    w->setStyleSheet(w->styleSheet());
}

void DefaultNodeAdapterBridge::triggerSetupAdaptiveUiRequest()
{
    Q_EMIT setupAdaptiveUiRequest();
}


/// ADAPTER
DefaultNodeAdapter::DefaultNodeAdapter(NodeHandleWeakPtr adaptee, NodeBox* parent)
    : NodeAdapter(adaptee, parent), bridge(this), wrapper_layout_(nullptr)
{
}

DefaultNodeAdapter::~DefaultNodeAdapter()
{
    clear();
}

void DefaultNodeAdapter::clear()
{
    bridge.disconnect();

    QtHelper::clearLayout(wrapper_layout_);

    for(QObject* cb : callbacks) {
        qt_helper::Call* call = dynamic_cast<qt_helper::Call*>(cb);
        if(call) {
            call->disconnect();
        }
        delete call;
        //cb->deleteLater();
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
void ui_updateColorParameter(param::ColorParameterWeakPtr color_p, QPointer<QPushButton> btn)
{
    assertGuiThread();
    auto p = color_p.lock();
    if(!p || !btn) {
        return;
    }
    btn->setStyleSheet(toColorSS(p->value()));
}

void model_updateColorParameter(param::ColorParameterWeakPtr color_p)
{
    assertGuiThread();
    auto p = color_p.lock();
    if(!p) {
        return;
    }

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
void ui_updatePathParameter(param::PathParameterWeakPtr path_p, QPointer<QLineEdit> path)
{
    assertGuiThread();
    auto p = path_p.lock();
    if(!p || !path) {
        return;
    }
    path->setText(QString::fromStdString(p->as<std::string>()));
}

void model_updatePathParameter(param::PathParameterWeakPtr path_p, QPointer<QLineEdit> path)
{
    assertNotGuiThread();
    auto p = path_p.lock();
    if(!p || !path) {
        return;
    }
    auto path_str = path->text().toStdString();
    p->set<std::string>(path_str);
}

void ui_updatePathParameterDialog(param::PathParameterWeakPtr path_p)
{
    assertGuiThread();
    auto p = path_p.lock();
    if(!p) {
        return;
    }
    QString filter = QString::fromStdString(p->filter());
    if(filter.isEmpty()) {
        filter = "All files (*.*)";
    }

    int flags = QFileDialog::DontUseNativeDialog;
    bool is_file = p->isFile();

    QString dir(p->as<std::string>().c_str());
    if(dir.startsWith("file://", Qt::CaseInsensitive)) {
        dir = dir.replace("file://", "", Qt::CaseInsensitive);
    }

    QString path;
    if(p->isOutput()) {
        if(is_file) {
            path = QFileDialog::getSaveFileName((QWidget*) 0, p->name().c_str(), dir, filter, (QString*) 0, (QFlags<QFileDialog::Option>) flags);
        } else {
            path = QFileDialog::getExistingDirectory((QWidget*) 0, p->name().c_str(), dir, (QFlags<QFileDialog::Option>) flags);
        }
    } else {
        if(is_file) {
            path = QFileDialog::getOpenFileName((QWidget*) 0, p->name().c_str(), dir, filter, (QString*) 0, (QFlags<QFileDialog::Option>) flags);
        } else {
            path = QFileDialog::getExistingDirectory((QWidget*) 0, p->name().c_str(), dir, (QFlags<QFileDialog::Option>) flags);
        }
    }

    if(!path.isEmpty()) {
        p->set(path.toStdString());
        //        cb();
    }
}

// INTERVAL ////////////////////
template <typename T, typename Widget>
void ui_updateIntervalParameter(param::IntervalParameterWeakPtr interval_p, QPointer<Widget> slider)
{
    assertGuiThread();
    auto p = interval_p.lock();
    if(!p || !slider) {
        return;
    }
    slider->setSpan(p->lower<T>(), p->upper<T>());
}

void model_updateIntIntervalParameter(param::IntervalParameterWeakPtr interval_p, QPointer<QxtSpanSlider> slider)
{
    assertNotGuiThread();
    auto p = interval_p.lock();
    if(!p || !slider) {
        return;
    }
    p->set(std::make_pair(slider->lowerValue(), slider->upperValue()));
}
void model_updateDoubleIntervalParameter(param::IntervalParameterWeakPtr interval_p, QPointer<DoubleSpanSlider> slider)
{
    assertNotGuiThread();
    auto p = interval_p.lock();
    if(!p || !slider) {
        return;
    }
    p->set(std::make_pair(slider->lowerDoubleValue(), slider->upperDoubleValue()));
}

template <typename T, typename Widget>
void ui_updateIntervalParameterScope(param::IntervalParameterWeakPtr interval_p, QPointer<Widget> slider)
{
    assertGuiThread();
    auto p = interval_p.lock();
    if(!p || !slider) {
        return;
    }
    slider->setRange(p->min<T>(), p->max<T>());
}

// SET ////////////////////
void ui_updateSetParameter(param::SetParameterWeakPtr set_p, QPointer<QComboBox> combo)
{
    assertGuiThread();
    auto p = set_p.lock();
    if(!p || !combo) {
        return;
    }
    int index = combo->findData(QString::fromStdString(p->getText()));
    if(index >= 0) {
        combo->setCurrentIndex(index);
    }
}
void model_updateSetParameter(param::SetParameterWeakPtr set_p, QPointer<QComboBox> combo)
{
    auto p = set_p.lock();
    if(!p || !combo) {
        return;
    }
    if(!combo->currentText().isEmpty()) {
        assertNotGuiThread();
        p->setByName(combo->currentText().toStdString());
    }
}

void ui_updateSetParameterScope(param::SetParameterWeakPtr set_p, QPointer<QComboBox> combo)
{
    assertGuiThread();
    auto p = set_p.lock();
    if(!p || !combo) {
        return;
    }

    int current = 0;
    combo->clear();
    std::string selected;
    try {
        selected = p->getText();
    } catch(const std::exception& e) {
        selected = "";
    }

    for(int i = 0; i < p->noParameters(); ++i) {
        std::string str = p->getText(i);
        combo->addItem(QString::fromStdString(str));

        if(str == selected) {
            current = i;
        }
    }
    combo->setCurrentIndex(current);
}

// BITSET ////////////////////
void ui_updateBitSetParameter(param::BitSetParameterWeakPtr bitset_p, QPointer<QCheckBox> item, const std::string& str)
{
    assertGuiThread();
    auto p = bitset_p.lock();
    if(!p || !item) {
        return;
    }
    item->setChecked(p->isSet(str));
}
void model_updateBitSetParameter(param::BitSetParameterWeakPtr bitset_p, QPointer<QCheckBox> item, const std::string& str)
{
    assertNotGuiThread();
    auto p = bitset_p.lock();
    if(!p || !item) {
        return;
    }
    p->setBitTo(str, item->isChecked());
}

// ANGLE ///////////////////////

double normalizeAngle(double a)
{
    double r = a;
    while(r < -M_PI) r += 2*M_PI;
    while(r >= M_PI) r -= 2*M_PI;
    return r;
}

double angleToDial(double angle)
{
    return (normalizeAngle(angle) + M_PI) * 4.0 * 180.0 / M_PI;
}

double dialToAngle(double dial)
{
    return normalizeAngle(dial  / 4.0 / 180.0 * M_PI - M_PI);
}

void model_updateAngleDialParameter(param::AngleParameterWeakPtr angle_p, QPointer<QDial> dial, QPointer<QDoubleSpinBox> spin)
{
    assertNotGuiThread();
    auto p = angle_p.lock();
    if(!p || !dial  || !spin) {
        return;
    }

    double angle = dialToAngle(dial->value());
    double min = p->min();
    double max = p->max();

    if(angle < min) {
        angle = min;
    } else if(angle > max) {
        angle = max;
    }

    p->set<double>(angle);
    spin->blockSignals(true);
    spin->setValue(angle);
    spin->blockSignals(false);
}
void model_updateAngleSpinParameter(param::AngleParameterWeakPtr angle_p, QPointer<QDial> dial, QPointer<QDoubleSpinBox> spin)
{
    assertNotGuiThread();
    auto p = angle_p.lock();
    if(!p || !dial  || !spin) {
        return;
    }

    double angle = spin->value();
    double min = p->min();
    double max = p->max();

    if(angle < min) {
        angle = min;
    } else if(angle > max) {
        angle = max;
    }

    p->set<double>(angle);
    dial->blockSignals(true);
    dial->setValue(angleToDial(p->as<double>()));
    dial->blockSignals(false);
}
void ui_updateAngleParameter(param::AngleParameterWeakPtr range_p, QPointer<QDial> dial, QPointer<QDoubleSpinBox> spin)
{
    assertGuiThread();
    auto p = range_p.lock();
    if(!p || !dial  || !spin) {
        return;
    }

    double angle = normalizeAngle(p->as<double>());
    double min = p->min();
    double max = p->max();

    if(angle < min) {
        angle = min;
    } else if(angle > max) {
        angle = max;
    }
    double val = angleToDial(angle);

    dial->setValue(val);
    spin->setValue(angle);
}

// PROGRESS ////////////////////
void ui_updateProgressParameter(param::OutputProgressParameterWeakPtr progress_p, QPointer<QProgressBar> bar)
{
    assertGuiThread();
    auto p = progress_p.lock();
    if(!p || !bar) {
        return;
    }
    bar->setValue(p->getProgress());
}
void ui_updateProgressScope(param::OutputProgressParameterWeakPtr progress_p, QPointer<QProgressBar> bar)
{
    assertGuiThread();
    auto p = progress_p.lock();
    if(!p || !bar) {
        return;
    }
    bar->setMaximum(p->getProgressMaximum());
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

void setDirection(QBoxLayout* layout, NodeHandleWeakPtr node)
{
    NodeHandlePtr n = node.lock();
    if(n) {
        layout->setDirection(n->getNodeState()->isFlipped() ? QHBoxLayout::RightToLeft : QHBoxLayout::LeftToRight);
    }
}

template <typename P>
void install(std::map<int, std::function<void(DefaultNodeAdapter*, csapex::param::Parameter::Ptr)> >& map)
{
    map[P().ID()] = [](DefaultNodeAdapter* a, csapex::param::Parameter::Ptr p) { a->setupParameter(std::dynamic_pointer_cast<P>(p)); };
}

}

void DefaultNodeAdapter::setupAdaptiveUi()
{
    NodeHandlePtr node_handle = node_.lock();
    if(!node_handle) {
        return;
    }

    auto node = node_handle->getNode().lock();
    if(!node) {
        return;
    }

    static std::map<int, std::function<void(DefaultNodeAdapter*, csapex::param::Parameter::Ptr)> > mapping_;
    if(mapping_.empty()) {
        install<param::TriggerParameter>(mapping_);
        install<param::ColorParameter>(mapping_);
        install<param::PathParameter>(mapping_);
        install<param::ValueParameter>(mapping_);
        install<param::RangeParameter>(mapping_);
        install<param::IntervalParameter>(mapping_);
        install<param::SetParameter>(mapping_);
        install<param::BitSetParameter>(mapping_);
        install<param::AngleParameter>(mapping_);

        install<param::OutputProgressParameter>(mapping_);
    }

    clear();

    current_layout_ = wrapper_layout_;

    std::vector<csapex::param::Parameter::Ptr> params = node->getParameters();

    GenericState::Ptr state = std::dynamic_pointer_cast<GenericState>(node->getParameterState());
    if(state) {
        state->parameter_set_changed->disconnectAll();
        state->parameter_set_changed->connect(std::bind(&DefaultNodeAdapterBridge::triggerSetupAdaptiveUiRequest, &bridge));
    }

    for(csapex::param::Parameter::Ptr p : params) {
        csapex::param::Parameter* parameter = p.get();

        if(!parameter->isEnabled()) {
            continue;
        }

        current_name_= parameter->name();
        current_display_name_ = current_name_;
        std::size_t separator_pos = current_name_.find_first_of('/');

        QBoxLayout* group_layout = nullptr;

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

                groupsboxes[group] = gb;

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
                hider->setHidden(hidden);

                gb_layout->addWidget(hider);

                wrapper_layout_->addWidget(gb);



                qt_helper::Call* call_trigger = new qt_helper::Call(std::bind(&DefaultNodeAdapterBridge::enableGroup, &bridge, std::bind(&QGroupBox::isChecked, gb), group));
                callbacks.push_back(call_trigger);
                QObject::connect(gb, SIGNAL(toggled(bool)), call_trigger, SLOT(call()));

                QObject::connect(gb, SIGNAL(toggled(bool)), hider, SLOT(setVisible(bool)));
            }
        }

        current_layout_ = new QHBoxLayout;
        setDirection(current_layout_, node_);
        node_handle->getNodeState()->flipped_changed->connect(std::bind(&setDirection, current_layout_, node_));

        // connect parameter input, if available
        InputPtr param_in = node_handle->getParameterInput(current_name_).lock();
        if(param_in) {
            Port* port = parent_->createPort(param_in, current_layout_);

            port->setVisible(p->isInteractive());
            parameter_connections_[param_in.get()] = p->interactive_changed.connect([port](csapex::param::Parameter*, bool i) { return port->setVisible(i); });
        }

        // generate UI element
        if(mapping_.find(p->ID()) != mapping_.end()) {
            mapping_[p->ID()](this, p);

        } else {
            current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet registered (value: " + type2name(p->type()) + ")").c_str()));
        }

        // connect parameter output, if available
        OutputPtr param_out = node_handle->getParameterOutput(current_name_).lock();
        if(param_out) {
            Port* port = parent_->createPort(param_out, current_layout_);

            port->setVisible(p->isInteractive());
            parameter_connections_[param_out.get()] = p->interactive_changed.connect([port](csapex::param::Parameter*, bool i) { return port->setVisible(i); });
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

qt_helper::Call * DefaultNodeAdapter::makeModelCall(std::function<void()> cb)
{
    qt_helper::Call* call = new qt_helper::Call([this, cb](){
        NodeHandlePtr node = node_.lock();
        if(node) {
            node->executionRequested(cb);
        }
    });
    callbacks.push_back(call);
    return call;
}

qt_helper::Call * DefaultNodeAdapter::makeUiCall(std::function<void()> cb)
{
    qt_helper::Call* call = new qt_helper::Call(cb);
    callbacks.push_back(call);
    return call;
}

qt_helper::Call * DefaultNodeAdapter::makePausedUiCall(std::function<void()> cb)
{
    qt_helper::Call* call = new qt_helper::Call([this, cb](){
        GraphFacade* g = parent_->getGraphView()->getGraphFacade();
        bool paused = g->isPaused();
        g->pauseRequest(true);
        cb();
        g->pauseRequest(paused);
    });
    callbacks.push_back(call);
    return call;
}

void DefaultNodeAdapter::setupParameter(param::TriggerParameterPtr trigger_p)
{
    QPointer<QPushButton> btn = new QPushButton(trigger_p->name().c_str());

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(trigger_p)));

    qt_helper::Call* call_trigger = makeModelCall(std::bind(&param::TriggerParameter::trigger, trigger_p));
    QObject::connect(btn, SIGNAL(clicked()), call_trigger, SLOT(call()));
}

void DefaultNodeAdapter::setupParameter(param::ColorParameterPtr color_p)
{
    QPointer<QPushButton> btn = new QPushButton;

    btn->setStyleSheet(toColorSS(color_p->value()));

    QHBoxLayout* sub = new QHBoxLayout;
    sub->addWidget(btn);
    current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(color_p)));

    // ui callback
    qt_helper::Call* call = makeUiCall(std::bind(&model_updateColorParameter, color_p));
    QObject::connect(btn, SIGNAL(clicked()), call, SLOT(call()));

    // model change -> ui
    bridge.connectInGuiThread(color_p->parameter_changed, std::bind(&ui_updateColorParameter, color_p, btn));
}

void DefaultNodeAdapter::setupParameter(param::PathParameterPtr path_p)
{
    QPointer<QLineEdit> path = new QLineEdit(path_p->as<std::string>().c_str());
    QPointer<QPushButton> select = new QPushButton("select");

    QHBoxLayout* sub = new QHBoxLayout;

    sub->addWidget(path);
    sub->addWidget(select);

    current_layout_->addLayout(QtHelper::wrap(current_display_name_, sub, new ParameterContextMenu(path_p)));

    // ui change -> model
    qt_helper::Call* call_set_path = makeModelCall(std::bind(&model_updatePathParameter, path_p, path));
    QObject::connect(path, SIGNAL(returnPressed()), call_set_path, SLOT(call()));

    qt_helper::Call* call_select = makePausedUiCall(std::bind(&ui_updatePathParameterDialog, path_p));
    QObject::connect(select, SIGNAL(clicked()), call_select, SLOT(call()));

    // model change -> ui
    bridge.connectInGuiThread(path_p->parameter_changed, std::bind(&ui_updatePathParameter, path_p, path));
}

void DefaultNodeAdapter::setupParameter(param::ValueParameterPtr value_p)
{
    ParameterAdapterPtr adapter(std::make_shared<ValueParameterAdapter>(value_p));

    adapter->executeCommand.connect(executeCommand);

    adapters_.push_back(adapter);

    adapter->doSetup(current_layout_, current_display_name_);
}

void DefaultNodeAdapter::setupParameter(param::RangeParameterPtr range_p)
{
    ParameterAdapterPtr adapter(std::make_shared<RangeParameterAdapter>(range_p));

    adapter->executeCommand.connect(executeCommand);

    adapters_.push_back(adapter);

    adapter->doSetup(current_layout_, current_display_name_);
}

void DefaultNodeAdapter::setupParameter(param::IntervalParameterPtr interval_p)
{
    if(interval_p->is<std::pair<int, int> >()) {
        const std::pair<int,int>& v = interval_p->as<std::pair<int,int> >();
        QPointer<QxtSpanSlider> slider = makeSpanSlider(current_layout_, current_display_name_,
                                               v.first, v.second, interval_p->min<int>(), interval_p->max<int>(),
                                               interval_p,
                                               new ParameterContextMenu(interval_p));

        // ui change -> model
        qt_helper::Call* call = makeModelCall(std::bind(&model_updateIntIntervalParameter, interval_p, slider));
        QObject::connect(slider, SIGNAL(lowerValueChanged(int)), call, SLOT(call()));
        QObject::connect(slider, SIGNAL(upperValueChanged(int)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(interval_p->parameter_changed, std::bind(&ui_updateIntervalParameter<int, QxtSpanSlider>, interval_p, slider));

        // parameter scope changed -> update slider interval
        bridge.connectInGuiThread(interval_p->scope_changed, std::bind(&ui_updateIntervalParameterScope<int, QxtSpanSlider>, interval_p, slider));

    } else if(interval_p->is<std::pair<double, double> >()) {
        const std::pair<double,double>& v = interval_p->as<std::pair<double,double> >();
        QPointer<DoubleSpanSlider> slider = makeDoubleSpanSlider(current_layout_, current_display_name_,
                                                           v.first, v.second, interval_p->min<double>(), interval_p->max<double>(), interval_p->step<double>(),
                                                           new ParameterContextMenu(interval_p));

        // ui change -> model
        qt_helper::Call* call = makeModelCall(std::bind(&model_updateDoubleIntervalParameter, interval_p, slider));
        QObject::connect(slider, SIGNAL(lowerValueChanged(int)), call, SLOT(call()));
        QObject::connect(slider, SIGNAL(upperValueChanged(int)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(interval_p->parameter_changed, std::bind(&ui_updateIntervalParameter<double, DoubleSpanSlider>, interval_p, slider));

        // parameter scope changed -> update slider interval
        bridge.connectInGuiThread(interval_p->scope_changed, std::bind(&ui_updateIntervalParameterScope<double, DoubleSpanSlider>, interval_p, slider));

    } else {
        current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet implemented (inverval: " + type2name(interval_p->type()) + ")").c_str()));
    }
}

void DefaultNodeAdapter::setupParameter(param::SetParameterPtr set_p)
{
    QPointer<QComboBox> combo = new QComboBox;

    ui_updateSetParameterScope(set_p, combo);
    current_layout_->addLayout(QtHelper::wrap(current_display_name_, combo, new ParameterContextMenu(set_p)));

    // ui change -> model
    qt_helper::Call* call = makeModelCall(std::bind(&model_updateSetParameter, set_p, combo));
    QObject::connect(combo, SIGNAL(currentIndexChanged(QString)), call, SLOT(call()));

    // model change -> ui
    bridge.connectInGuiThread(set_p->parameter_changed, std::bind(&ui_updateSetParameter, set_p, combo));

    // parameter scope changed -> update slider interval
    bridge.connectInGuiThread(set_p->scope_changed, std::bind(&ui_updateSetParameterScope, set_p, combo));

}

void DefaultNodeAdapter::setupParameter(param::BitSetParameterPtr bitset_p)
{
    QPointer<QGroupBox> group = new QGroupBox(current_name_.c_str());
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
        qt_helper::Call* call = makeModelCall(std::bind(&model_updateBitSetParameter, bitset_p, item, str));
        QObject::connect(item, SIGNAL(toggled(bool)), call, SLOT(call()));

        // model change -> ui
        bridge.connectInGuiThread(bitset_p->parameter_changed, std::bind(&ui_updateBitSetParameter, bitset_p, item, str));
    }

    current_layout_->addWidget(group);
}


void DefaultNodeAdapter::setupParameter(param::AngleParameterPtr angle_p)
{
    QLabel* label = new QLabel(angle_p->name().c_str());

    ParameterContextMenu* context_handler = new ParameterContextMenu(angle_p);
    context_handler->addAction(new QAction("set -π", context_handler), [angle_p](){
        angle_p->set(-M_PI);
    });
    context_handler->addAction(new QAction("set -π/2", context_handler), [angle_p](){
        angle_p->set(-M_PI_2);
    });
    context_handler->addAction(new QAction("set 0", context_handler), [angle_p](){
        angle_p->set(0.0);
    });
    context_handler->addAction(new QAction("set +π/2", context_handler), [angle_p](){
        angle_p->set(M_PI_2);
    });
    context_handler->addAction(new QAction("set +π", context_handler), [angle_p](){
        angle_p->set(M_PI-1e-9);
    });
    context_handler->setParent(label);

    label->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(label, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));

    current_layout_->addWidget(label);

    QPointer<QDial> dial = new QDial;
    dial->setMinimum(0);
    dial->setMaximum(360.0 * 4);
    dial->setWrapping(true);
    dial->setValue(angleToDial(angle_p->as<double>()));

    dial->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(dial, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));

    current_layout_->addWidget(dial);

    QPointer<QDoubleSpinBox> spin = new QDoubleSpinBox;
    spin->setMinimum(-M_PI);
    spin->setMaximum(M_PI);
    spin->setDecimals(5);
    spin->setSingleStep(0.001);
    spin->setValue(angle_p->as<double>());

    current_layout_->addWidget(spin);

    // ui change -> model
    qt_helper::Call* call_dial = makeModelCall(std::bind(&model_updateAngleDialParameter, angle_p, dial, spin));
    QObject::connect(dial, SIGNAL(valueChanged(int)), call_dial, SLOT(call()));

    qt_helper::Call* call_spin = makeModelCall(std::bind(&model_updateAngleSpinParameter, angle_p, dial, spin));
    QObject::connect(spin, SIGNAL(valueChanged(double)), call_spin, SLOT(call()));

    // model change -> ui
    bridge.connectInGuiThread(angle_p->parameter_changed, std::bind(&ui_updateAngleParameter, angle_p, dial, spin));
}

void DefaultNodeAdapter::setupParameter(param::OutputProgressParameterPtr progress)
{
    QPointer<QProgressBar> bar = new QProgressBar;
    bar->setValue(progress->getProgress());
    bar->setMaximum(progress->getProgressMaximum());
    bar->setFormat(QString::fromStdString(progress->name()) + ": %p%");
    current_layout_->addWidget(bar);

    // model change -> ui
    bridge.connectInGuiThread(progress->parameter_changed, std::bind(&ui_updateProgressParameter, progress, bar));

    // parameter scope changed -> update slider interval
    bridge.connectInGuiThread(progress->scope_changed, std::bind(&ui_updateProgressScope, progress, bar));
}


void DefaultNodeAdapter::stop()
{
    NodeAdapter::stop();
    bridge.disconnect();
}
/// MOC
#include "../../../include/csapex/view/node/moc_default_node_adapter.cpp"
