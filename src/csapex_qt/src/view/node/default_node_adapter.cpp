/// HEADER
#include <csapex/view/node/default_node_adapter.h>

/// PROJECT
#include <csapex/model/generic_state.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/param/angle_parameter.h>
#include <csapex/param/bitset_parameter.h>
#include <csapex/param/color_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/param/output_progress_parameter.h>
#include <csapex/param/path_parameter.h>
#include <csapex/param/range_parameter.h>
#include <csapex/param/set_parameter.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/param/angle_param_adapter.h>
#include <csapex/view/param/bitset_param_adapter.h>
#include <csapex/view/param/color_param_adapter.h>
#include <csapex/view/param/interval_param_adapter.h>
#include <csapex/view/param/output_progress_param_adapter.h>
#include <csapex/view/param/output_text_param_adapter.h>
#include <csapex/view/param/path_param_adapter.h>
#include <csapex/view/param/range_param_adapter.h>
#include <csapex/view/param/set_param_adapter.h>
#include <csapex/view/param/value_param_adapter.h>
#include <csapex/view/param/trigger_param_adapter.h>
#include <csapex/view/utility/qsignal_relay.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/widgets/port.h>

/// SYSTEM
#include <functional>
#include <QPointer>
#include <QApplication>
#include <QGroupBox>
#include <QPushButton>

using namespace csapex;
using namespace csapex::param;

namespace {

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

void DefaultNodeAdapterBridge::connectInGuiThread(slim_signal::Signal<void (Parameter *)> &signal,
                                                  std::function<void ()> cb)
{
    // cb should be executed in the gui thread
    connections.push_back(signal.connect(std::bind(&DefaultNodeAdapterBridge::modelCallback, this, cb)));
}

void DefaultNodeAdapterBridge::disconnect()
{
    for(slim_signal::Connection& c : connections) {
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
    NodeFacadePtr node_handle = parent_->node_.lock();
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
DefaultNodeAdapter::DefaultNodeAdapter(NodeFacadePtr adaptee, NodeBox* parent)
    : NodeAdapter(adaptee, parent), bridge(this), wrapper_layout_(nullptr)
{
}

DefaultNodeAdapter::~DefaultNodeAdapter()
{
    clear();
    delete wrapper_layout_;
    wrapper_layout_ = nullptr;
}

void DefaultNodeAdapter::clear()
{
    bridge.disconnect();

    if(wrapper_layout_) {
        QtHelper::clearLayout(wrapper_layout_);
    }

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

void setDirection(QBoxLayout* layout, NodeFacadePtr node)
{
    layout->setDirection(node->getNodeState()->isFlipped() ? QHBoxLayout::RightToLeft : QHBoxLayout::LeftToRight);
}

template <typename P, typename Adapter = void>
struct install
{
    static void execute(std::map<int, std::function<void(DefaultNodeAdapter*, Parameter::Ptr)> >& map)
    {
        map[P().ID()] = [](DefaultNodeAdapter* a, Parameter::Ptr p) {
            a->setupParameter<P, Adapter>(std::dynamic_pointer_cast<P>(p));
        };
    }
};

}

void DefaultNodeAdapter::setupAdaptiveUi()
{
    NodeFacadePtr node_facade = node_.lock();
    if(!node_facade) {
        return;
    }

    static std::map<int, std::function<void(DefaultNodeAdapter*, Parameter::Ptr)> > mapping_;
    if(mapping_.empty()) {
        install<TriggerParameter, TriggerParameterAdapter>::execute(mapping_);

        install<ColorParameter, ColorParameterAdapter>::execute(mapping_);
        install<PathParameter, PathParameterAdapter>::execute(mapping_);
        install<ValueParameter, ValueParameterAdapter>::execute(mapping_);
        install<RangeParameter, RangeParameterAdapter>::execute(mapping_);
        install<IntervalParameter, IntervalParameterAdapter>::execute(mapping_);
        install<SetParameter, SetParameterAdapter>::execute(mapping_);
        install<BitSetParameter, BitSetParameterAdapter>::execute(mapping_);
        install<AngleParameter, AngleParameterAdapter>::execute(mapping_);

        install<OutputTextParameter, OutputTextParameterAdapter>::execute(mapping_);
        install<OutputProgressParameter, OutputProgressParameterAdapter>::execute(mapping_);
    }

    clear();

    current_layout_ = wrapper_layout_;

    std::vector<Parameter::Ptr> params = node_facade->getParameters();

    node_facade->parameter_set_changed.disconnectAll();
    node_facade->parameter_set_changed.connect(std::bind(&DefaultNodeAdapterBridge::triggerSetupAdaptiveUiRequest, &bridge));

    for(Parameter::Ptr p : params) {
        Parameter* parameter = p.get();

        if(!parameter->isEnabled() || parameter->isHidden()) {
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
                QObject::connect(gb, &QGroupBox::toggled, call_trigger, &qt_helper::Call::call);

                QObject::connect(gb, &QGroupBox::toggled, hider, &QFrame::setVisible);
            }
        }

        QPointer<QHBoxLayout> layout_ptr(new QHBoxLayout);
        current_layout_ = layout_ptr;
        setDirection(current_layout_, node_facade);
        node_facade->getNodeState()->flipped_changed.connect([this, layout_ptr](){
            if(!layout_ptr.isNull()) {
                if(auto node = node_.lock()) {
                    setDirection(layout_ptr, node);
                }
            }
        });

        // connect parameter input, if available
        ConnectorPtr param_in = node_facade->getParameterInput(current_name_);
        if(param_in) {
            QPointer<Port> port = parent_->createPort(param_in, current_layout_);

            port->setVisible(p->isInteractive());
            parameter_connections_[p.get()].push_back(p->interactive_changed.connect([port](Parameter*, bool i) {
                                                          if(port.isNull()) {
                                                              return;
                                                          }
                                                          return port->setVisible(i);
                                                      }));
        }

        // generate UI element
        if(mapping_.find(p->ID()) != mapping_.end()) {
            mapping_[p->ID()](this, p);

        } else {
            current_layout_->addWidget(new QLabel((current_name_ + "'s type is not yet registered (value: " + type2name(p->type()) + ")").c_str()));
        }

        // connect parameter output, if available
        ConnectorPtr param_out = node_facade->getParameterOutput(current_name_);
        if(param_out) {
            QPointer<Port> port = parent_->createPort(param_out, current_layout_);

            port->setVisible(p->isInteractive());
            parameter_connections_[p.get()].push_back(p->interactive_changed.connect([port](Parameter*, bool i) {
                                                          if(port.isNull()) {
                                                              return;
                                                          }
                                                          return port->setVisible(i);
                                                      }));
        }

        QString tooltip = QString::fromStdString(p->description().toString());
        if(tooltip.isEmpty()){
            setTooltip(current_layout_, QString::fromStdString(p->getUUID().getAbsoluteUUID().getFullName()));
        } else {
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


template <typename Parameter, typename Adapter>
void DefaultNodeAdapter::setupParameter(std::shared_ptr<Parameter> p)
{
    ParameterAdapterPtr adapter(std::make_shared<Adapter>(p));
    adapters_.push_back(adapter);
    adapter->executeCommand.connect(executeCommand);

    adapter->doSetup(current_layout_, current_display_name_);

    parameter_connections_[p.get()].push_back(p->destroyed.connect([this, adapter](param::Parameter* p) {
        auto pos = std::find(adapters_.begin(), adapters_.end(), adapter);
        if(pos != adapters_.end()) {
            adapters_.erase(pos);
        }
        parameter_connections_.erase(p);
    }));
}


void DefaultNodeAdapter::stop()
{
    NodeAdapter::stop();
    bridge.disconnect();
}
/// MOC
#include "../../../include/csapex/view/node/moc_default_node_adapter.cpp"
