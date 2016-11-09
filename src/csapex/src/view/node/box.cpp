/// HEADER
#include <csapex/view/node/box.h>

/// COMPONENT
#include "ui_box.h"
#include <csapex/model/node.h>
#include <csapex/factory/node_factory.h>
#include <csapex/command/command_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/variadic_io.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/command/meta.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/node/node_adapter.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/core/settings.h>
#include <csapex/view/widgets/port.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/model/graph_facade.h>
#include <csapex/view/widgets/meta_port.h>
#include <csapex/view/utility/qt_helper.hpp>

/// SYSTEM
#include <QMenu>
#include <QTimer>
#include <QPainter>
#include <QContextMenuEvent>
#include <iostream>
#include <QSizeGrip>
#include <QThread>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace csapex;

NodeBox::NodeBox(Settings& settings, NodeHandlePtr handle, NodeWorker::Ptr worker, QIcon icon, GraphView* parent)
    : parent_(parent), ui(nullptr), grip_(nullptr), settings_(settings), node_handle_(handle), node_worker_(worker), adapter_(nullptr), icon_(icon),
      info_exec(nullptr), info_compo(nullptr), info_thread(nullptr), info_frequency(nullptr), info_error(nullptr), initialized_(false),
      frequency_timer_(nullptr)
{
    QObject::connect(this, SIGNAL(updateVisualsRequest()), this, SLOT(updateVisuals()));

    setVisible(false);
}

NodeBox::NodeBox(Settings& settings, NodeHandlePtr handle, QIcon icon, GraphView* parent)
    : NodeBox(settings, handle, nullptr, icon, parent)
{
}

void NodeBox::setAdapter(NodeAdapter::Ptr adapter)
{
    adapter_ = adapter;

    if(adapter->isResizable()) {
        grip_ = new QSizeGrip(this);
    }
}

NodeBox::~NodeBox()
{
    destruct();

    for(QObject* child : children()) {
        delete child;
    }

    delete ui;
    initialized_ = false;

    delete frequency_timer_;
}

void NodeBox::destruct()
{
    QObject::disconnect(this);

    node_worker_.reset();
    adapter_.reset();
}

void NodeBox::setupUi()
{
    if(!info_exec) {
        info_exec = new QLabel;
        info_exec->setProperty("exec", true);
        ui->infos->addWidget(info_exec);
    }
    if(!info_compo) {
        info_compo = new QLabel;
        info_compo->setProperty("component", true);
        ui->infos->addWidget(info_compo);
    }

    if(!info_thread) {
        info_thread = new QLabel;
        info_thread->setProperty("threadgroup", true);
        ui->infos->addWidget(info_thread);
    }

    if(!info_frequency) {
        info_frequency = new QLabel;
        info_frequency->setProperty("frequency", true);
        ui->infos->addWidget(info_frequency);
    }

    if(!info_error) {
        info_error = new QLabel;
        info_error->setText("<img src=':/error.png' />");
        info_error->setProperty("error", true);
        info_error->setVisible(false);
        ui->infos->addWidget(info_error);
    }

    if(dynamic_cast<VariadicBase*>(getNode())) {
        AUUID parent = getNodeHandle()->getUUID().getAbsoluteUUID();
        if(dynamic_cast<VariadicInputs*>(getNode())) {
            MetaPort* meta_port = new MetaPort(ConnectorType::INPUT, parent);
            QObject::connect(meta_port, &MetaPort::createPortRequest, this, &NodeBox::createPortRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &NodeBox::createPortAndConnectRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &NodeBox::createPortAndMoveRequest);
            ui->input_panel->layout()->addWidget(meta_port);
        }
        if(dynamic_cast<VariadicOutputs*>(getNode())) {
            MetaPort* meta_port = new MetaPort(ConnectorType::OUTPUT, parent);
            QObject::connect(meta_port, &MetaPort::createPortRequest, this, &NodeBox::createPortRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &NodeBox::createPortAndConnectRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &NodeBox::createPortAndMoveRequest);
            ui->output_panel->layout()->addWidget(meta_port);
        }
        if(dynamic_cast<VariadicSlots*>(getNode())) {
            MetaPort* meta_port = new MetaPort(ConnectorType::SLOT_T, parent);
            QObject::connect(meta_port, &MetaPort::createPortRequest, this, &NodeBox::createPortRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &NodeBox::createPortAndConnectRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &NodeBox::createPortAndMoveRequest);
            ui->slot_panel->layout()->addWidget(meta_port);
        }
        if(dynamic_cast<VariadicEvents*>(getNode())) {
            MetaPort* meta_port = new MetaPort(ConnectorType::EVENT, parent);
            QObject::connect(meta_port, &MetaPort::createPortRequest, this, &NodeBox::createPortRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &NodeBox::createPortAndConnectRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &NodeBox::createPortAndMoveRequest);
            ui->event_panel->layout()->addWidget(meta_port);
        }
    }


    NodeState* state = node_handle_.lock()->getNodeState().get();
    connections_.push_back(state->flipped_changed->connect(std::bind(&NodeBox::triggerFlipSides, this)));
    connections_.push_back(state->minimized_changed->connect(std::bind(&NodeBox::triggerMinimized, this)));
    connections_.push_back(state->active_changed->connect([this, state](){
        setProperty("active", state->isActive());
        updateVisualsRequest();
    }));
    connections_.push_back(state->color_changed->connect(std::bind(&NodeBox::changeColor, this)));
    connections_.push_back(state->pos_changed->connect(std::bind(&NodeBox::updatePosition, this)));

    connections_.push_back(settings_.settingsChanged.connect([this](const std::string& name) {
        if(name == "debug") {
            changeColor();
        }
    }));

    updateStylesheetColor();

    ui->header->setAlignment(Qt::AlignTop);
    ui->content->setAlignment(Qt::AlignTop);

    if(adapter_) {
        adapter_->doSetupUi(ui->content);
    }
    setAutoFillBackground(false);

    setAttribute( Qt::WA_TranslucentBackground, true );
    setAttribute(Qt::WA_NoSystemBackground, true);
    //    setBackgroundMode (Qt::NoBackground, true);

    updateVisuals();

    Q_EMIT changed(this);
}

void NodeBox::construct()
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }

    ui = new Ui::Box;
    ui->setupUi(this);

    ui->input_layout->addSpacerItem(new QSpacerItem(16, 0));
    ui->output_layout->addSpacerItem(new QSpacerItem(16, 0));

    ui->enablebtn->setCheckable(true);
    ui->enablebtn->setChecked(nh->getNodeState()->isEnabled());

    QSize size(16, 16);
    ui->icon->setPixmap(icon_.pixmap(size));

    setFocusPolicy(Qt::ClickFocus);

    const UUID& uuid = nh->getUUID();
    setToolTip(QString::fromStdString(uuid.getFullName()));

    setObjectName(QString::fromStdString(uuid.getFullName()));

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    if(grip_) {
        grip_->installEventFilter(this);
    }

    setLabel(nh->getNodeState()->getLabel());

    QObject::connect(ui->enablebtn, &QCheckBox::toggled, this, &NodeBox::toggled);


    connections_.emplace_back(nh->nodeStateChanged.connect([this]() { nodeStateChanged(); }));
    QObject::connect(this, &NodeBox::nodeStateChanged, this, &NodeBox::nodeStateChangedEvent, Qt::QueuedConnection);

    connections_.emplace_back(nh->connectorCreated.connect([this](ConnectablePtr c) { registerEvent(c.get()); }));
    connections_.emplace_back(nh->connectorRemoved.connect([this](ConnectablePtr c) { unregisterEvent(c.get()); }));

    NodeWorkerPtr worker = node_worker_.lock();
    if(worker) {
        connections_.emplace_back(worker->destroyed.connect([this](){ destruct(); }));

        enabledChangeEvent(worker->isProcessingEnabled());
        //    nh->enabled.connect([this](bool e){ enabledChange(e); });
        QObject::connect(this, &NodeBox::enabledChange, this, &NodeBox::enabledChangeEvent, Qt::QueuedConnection);

        connections_.emplace_back(worker->notification.connect([this](Notification){ updateVisualsRequest(); }));
    }


    for(auto input : nh->getExternalInputs()) {
        registerInputEvent(input.get());
    }
    for(auto output : nh->getExternalOutputs()) {
        registerOutputEvent(output.get());
    }

    setupUi();

    installEventFilter(this);
}


Node* NodeBox::getNode() const
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return nullptr;
    }
    return nh->getNode().lock().get();
}

NodeWorker* NodeBox::getNodeWorker() const
{
    NodeWorkerPtr worker = node_worker_.lock();
    if(!worker) {
        return nullptr;
    }
    return worker.get();
}

NodeHandle* NodeBox::getNodeHandle() const
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return nullptr;
    }
    return nh.get();
}

NodeAdapter::Ptr NodeBox::getNodeAdapter() const
{
    return adapter_;
}

GraphView* NodeBox::getGraphView() const
{
    return parent_;
}


void NodeBox::updateBoxInformation(Graph* graph)
{
    updateComponentInformation(graph);
    updateThreadInformation();
    updateFrequencyInformation();
}

namespace {
void setStyleForId(QLabel* label, int id) {
    // set color using HSV rotation
    double hue =  (id * 77) % 360;
    double r = 0, g = 0, b = 0;
    __HSV2RGB__(hue, 1., 1., r, g, b);
    double fr = 0, fb = 0, fg = 0;
    double min = std::min(b, std::min(g, r));
    double max = std::max(b, std::max(g, r));
    if(min < 100 && max < 100) {
        fr = fb = fg = 255;
    }
    std::stringstream ss;
    ss << "QLabel { background-color : rgb(" << r << "," << g << "," << b << "); color: rgb(" << fr << "," << fg << "," << fb << ");}";
    label->setStyleSheet(ss.str().c_str());
}
}

void NodeBox::updateComponentInformation(Graph* graph)
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }

    if(settings_.get("debug", false)) {
        changeColor();
    }

    if(!settings_.get<bool>("display-graph-components", false)) {
        info_compo->setVisible(false);
        return;
    } else {
        info_compo->setVisible(true);
    }

    if(info_compo->isVisible()) {
        int compo = graph->getComponent(nh->getUUID());
        int depth = graph->getDepth(nh->getUUID());
        std::stringstream info;
        info << "C:" << compo;
        info << "D:" << depth;
        info_compo->setText(info.str().c_str());

        const auto& chara = nh->getVertex()->getNodeCharacteristics();

        QString tooltip("characteristics: ");
//        tooltip += QString("vertex separator: ") + (chara.is_vertex_separator ? "yes" : "no") + ", ";
        tooltip += QString("joining vertex: ") + (chara.is_joining_vertex ? "yes" : "no") + ", ";
        tooltip += QString("leading to joining vertex: ") + (chara.is_leading_to_joining_vertex ? "yes" : "no") + ", ";
        tooltip += QString("combined by joining vertex: ") + (chara.is_combined_by_joining_vertex ? "yes" : "no") + ", ";
        tooltip += QString("joining vertex counterpart: ") + (chara.is_joining_vertex_counterpart ? "yes" : "no");
        info_compo->setToolTip(tooltip);

        setStyleForId(info_compo, compo);
    }
}

void NodeBox::updateThreadInformation()
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }

    if(!settings_.get<bool>("display-threads", false)) {
        info_thread->setVisible(false);
        return;
    } else {
        info_thread->setVisible(true);
    }

    if(info_thread->isVisible()) {
        NodeStatePtr state = nh->getNodeState();
        int id = state->getThreadId();
        std::stringstream info;
        if(settings_.get<bool>("threadless")) {
            info << "<i><b><small>threadless</small></b></i>";
            info_thread->setStyleSheet("QLabel { background-color : rgb(0,0,0); color: rgb(255,255,255);}");
        } else if(id < 0) {
            info << "T:" << -id;
            setStyleForId(info_thread, id);
        } else if(id == 0) {
            info << "<i><b><small>private</small></b></i>";
            info_thread->setStyleSheet("QLabel { background-color : rgb(255,255,255); color: rgb(0,0,0);}");
        } else {
            info << state->getThreadName();
            setStyleForId(info_thread, id);
        }
        info_thread->setText(info.str().c_str());
    }
}


void NodeBox::updateFrequencyInformation()
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }

    if(!settings_.get<bool>("display-frequencies", false)) {
        info_frequency->setVisible(false);

        if(frequency_timer_ && frequency_timer_->isActive()) {
            frequency_timer_->stop();
        }
        return;
    } else {
        info_frequency->setVisible(true);
        if(!frequency_timer_) {
            frequency_timer_ = new QTimer;
            QObject::connect(frequency_timer_, &QTimer::timeout, this, &NodeBox::updateFrequencyInformation);
        }

        if(!frequency_timer_->isActive()){
            frequency_timer_->start(1000);
        }
    }

    if(info_frequency->isVisible()) {
        const Rate& rate = nh->getRate();
        double f = rate.getEffectiveFrequency();
        std::stringstream info;
        info << "<i><b>" << std::setprecision(2) << f << "Hz</b></i>";
        info_frequency->setText(info.str().c_str());
    }
}

void NodeBox::contextMenuEvent(QContextMenuEvent* e)
{
    Q_EMIT showContextMenuForBox(this, e->globalPos());
}

QBoxLayout* NodeBox::getInputLayout()
{
    return ui->input_layout;
}

QBoxLayout* NodeBox::getOutputLayout()
{
    return ui->output_layout;
}

QBoxLayout* NodeBox::getSlotLayout()
{
    return ui->slot_layout;
}

QBoxLayout* NodeBox::getEventLayout()
{
    return ui->signal_layout;
}

bool NodeBox::isError() const
{
    NodeWorkerPtr worker = node_worker_.lock();
    if(!worker) {
        return false;
    }
    return worker->isError();
}
ErrorState::ErrorLevel NodeBox::errorLevel() const
{
    NodeWorkerPtr worker = node_worker_.lock();
    if(!worker) {
        return ErrorState::ErrorLevel::NONE;
    }
    return worker->errorLevel();
}
std::string NodeBox::errorMessage() const
{
    NodeWorkerPtr worker = node_worker_.lock();
    if(!worker) {
        return "";
    }
    return worker->errorMessage();
}

void NodeBox::setLabel(const std::string& label)
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }
    NodeStatePtr state = nh->getNodeState();

    apex_assert_hard(state);
    state->setLabel(label);
    ui->label->setText(label.c_str());
    ui->label->setToolTip(label.c_str());
}

void NodeBox::setLabel(const QString &label)
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }
    NodeStatePtr state = nh->getNodeState();
    state->setLabel(label.toStdString());
    ui->label->setText(label);
}

std::string NodeBox::getLabel() const
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return "";
    }
    NodeStatePtr state = nh->getNodeState();
    return state->getLabel();
}

void NodeBox::registerEvent(Connectable* c)
{
    if(c->isOutput()) {
        registerOutputEvent(dynamic_cast<Output*>(c));
    } else {
        registerInputEvent(dynamic_cast<Input*>(c));
    }
}

void NodeBox::unregisterEvent(Connectable*)
{
}

void NodeBox::registerInputEvent(Input* /*in*/)
{
    Q_EMIT changed(this);
}

void NodeBox::registerOutputEvent(Output* /*out*/)
{
    Q_EMIT changed(this);
}

void NodeBox::resizeEvent(QResizeEvent */*e*/)
{
    Q_EMIT changed(this);
}

void NodeBox::init()
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }

    NodeStatePtr state = nh->getNodeState();
    updatePosition();
    (*state->pos_changed)();

    setVisible(true);
}

Port* NodeBox::createPort(ConnectableWeakPtr connector, QBoxLayout *layout)
{
    apex_assert_hard(QApplication::instance()->thread() == QThread::currentThread());

    Port* port = new Port(connector);

    port->setFlipped(isFlipped());
    port->setMinimizedSize(isMinimizedSize());

    QObject::connect(this, SIGNAL(minimized(bool)), port, SLOT(setMinimizedSize(bool)));
    QObject::connect(this, SIGNAL(flipped(bool)), port, SLOT(setFlipped(bool)));

    ConnectablePtr adaptee = port->getAdaptee().lock();
    apex_assert_hard(adaptee == connector.lock());

    if(dynamic_cast<VariadicBase*>(getNode())) {
        std::vector<MetaPort*> metas;
        for(int i = 0; i < layout->count();) {
            MetaPort* meta = dynamic_cast<MetaPort*>(layout->itemAt(i)->widget());
            if(meta) {
                metas.push_back(meta);
                layout->removeWidget(meta);
            } else {
                ++i;
            }
        }

        layout->addWidget(port);

        for(MetaPort* meta : metas) {
            layout->addWidget(meta);
        }
    } else {
        layout->addWidget(port);
    }

    port_map_[adaptee->getUUID()] = port;

    portAdded(port);

    return port;
}

void NodeBox::removePort(ConnectableWeakPtr connector)
{
    ConnectablePtr adaptee = connector.lock();
    apex_assert_hard(adaptee);

    Port* port = port_map_.at(adaptee->getUUID());
    if(port) {
        port->deleteLater();
    }

    port_map_.erase(adaptee->getUUID());

    portRemoved(port);
}

bool NodeBox::eventFilter(QObject* o, QEvent* e)
{
    if(o == this) {
        if(e->type() == QEvent::MouseButtonDblClick) {
            if(hasSubGraph()) {
                Q_EMIT showSubGraphRequest(getSubGraph()->getAbsoluteUUID());
                return true;
            }
        }
        return false;
    }

    if(ui && o == ui->label) {
        QMouseEvent* em = dynamic_cast<QMouseEvent*>(e);
        if(e->type() == QEvent::MouseButtonDblClick && em->button() == Qt::LeftButton) {
            Q_EMIT renameRequest(this);
            e->accept();

            return true;
        }
    } else if(grip_ && o == grip_) {
        if(e->type() == QEvent::MouseButtonPress) {
            startResize();
        } else if(e->type() == QEvent::MouseButtonRelease) {
            stopResize();
        }
    }

    return false;
}

void NodeBox::startResize()
{
    adapter_->setManualResize(true);
}
void NodeBox::stopResize()
{
    adapter_->setManualResize(false);
}

void NodeBox::enabledChangeEvent(bool val)
{
    setProperty("disabled", !val);

    ui->enablebtn->blockSignals(true);
    ui->enablebtn->setChecked(val);
    ui->enablebtn->blockSignals(false);

    refreshTopLevelStylesheet();
}

QString NodeBox::getNodeState()
{
    NodeWorkerPtr worker = node_worker_.lock();
    if(!worker) {
        return "";
    }

    QString state;
    switch(worker->getState()) {
    case NodeWorker::State::IDLE:
        state = "idle"; break;
    case NodeWorker::State::ENABLED:
        state = "enabled"; break;
    case NodeWorker::State::FIRED:
        state = "fired"; break;
    case NodeWorker::State::PROCESSING:
        state = "processing"; break;
    default:
        state = "unknown"; break;
    }

    return state;
}

void NodeBox::paintEvent(QPaintEvent* /*e*/)
{
    NodeWorkerPtr worker = node_worker_.lock();
    if(!adapter_) {
        return;
    }
    QString state = getNodeState();
    QString state_text;
    QString transition_state;
    if(worker) {
        NodeHandlePtr handle = worker->getNodeHandle();
        OutputTransition* ot = handle->getOutputTransition();
        InputTransition* it = handle->getInputTransition();

        transition_state += ", it: ";
        transition_state += it->isEnabled() ? "enabled" : "disabled";
        transition_state += ", ot: ";
        transition_state += ot->isEnabled() ? "enabled" : "disabled";
        state_text = "<img src=\":/node_";
        state_text += state + ".png\" />";
        if(handle->getNodeState()->isMuted()) {
            state_text += "<img src=\":/muted.png\" />";
        }
    }

    info_exec->setText(state_text);
    info_exec->setToolTip(state + transition_state);

    bool is_error = false;
    bool is_warn = false;
    if(worker) {
        is_error = worker->isError() && worker->errorLevel() == ErrorState::ErrorLevel::ERROR;
        is_warn = worker->isError() && worker->errorLevel() == ErrorState::ErrorLevel::WARNING;
    }

    bool error_change = ui->boxframe->property("error").toBool() != is_error;
    bool warning_change = ui->boxframe->property("warning").toBool() != is_warn;

    setProperty("error", is_error);
    setProperty("warning", is_warn);

    if(error_change || warning_change) {
        if(is_error) {
            QString msg = QString::fromStdString(worker->errorMessage());
            setToolTip(msg);
            info_error->setToolTip(msg);
            info_error->setVisible(true);
        } else if(is_warn) {
            QString msg = QString::fromStdString(worker->errorMessage());
            setToolTip(msg);
            info_error->setToolTip(msg);
            info_error->setVisible(true);
        } else {
            setToolTip(ui->label->text());
            info_error->setVisible(false);
        }

        refreshTopLevelStylesheet();
    }

    if(!initialized_) {
        adjustSize();
        initialized_ = true;
    }
}

void NodeBox::moveEvent(QMoveEvent* e)
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }

    eventFilter(this, e);
}

bool NodeBox::isSelected() const
{
    return property("focused").toBool();
}

void NodeBox::setSelected(bool selected)
{
    setProperty("focused",selected);
    for(Port* port : findChildren<Port*>()) {
        port->setProperty("focused",selected);
    }
    refreshTopLevelStylesheet();
}

void NodeBox::keyPressEvent(QKeyEvent *)
{

}

void NodeBox::stop()
{
    QObject::disconnect(this);
    adapter_->stop();
}

void NodeBox::getInformation()
{
    Q_EMIT helpRequest(this);
}

void NodeBox::refreshStylesheet()
{
    apex_assert_hard(QThread::currentThread() == QApplication::instance()->thread());
    for(auto* child : children()) {
        if(QWidget* widget = dynamic_cast<QWidget*>(child)) {
            widget->style()->polish(widget);
        }
    }

    style()->polish(this);
    update();
}

void NodeBox::refreshTopLevelStylesheet()
{
    ui->boxframe->style()->polish(ui->boxframe);
    style()->polish(this);
    update();
}

void NodeBox::showProfiling(bool profiling)
{
    NodeWorkerPtr node = node_worker_.lock();
    if(node->isProfiling() != profiling) {
        node->setProfiling(profiling);
    }
}

void NodeBox::killContent()
{
    NodeWorkerPtr worker = node_worker_.lock();
    if(!worker) {
        return;
    }
    worker->killExecution();
}

void NodeBox::triggerFlipSides()
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }

    NodeStatePtr state = nh->getNodeState();
    bool flip = state->isFlipped();
    Q_EMIT flipped(flip);
}

void NodeBox::triggerMinimized()
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }

    NodeStatePtr state = nh->getNodeState();
    bool minimize = state->isMinimized();
    Q_EMIT minimized(minimize);
}

void NodeBox::updateStylesheetColor()
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }
    NodeStatePtr state = nh->getNodeState();

    QColor text_color = Qt::black;

    int r, g, b;
    if(settings_.get("debug", false)) {
        r = 0; g = 0; b = 0;
        if(NodeHandlePtr nh = node_handle_.lock()) {
            graph::VertexPtr vertex = nh->getVertex();

            const auto& characteristics = vertex->getNodeCharacteristics();
            if(characteristics.is_joining_vertex) {
                r = 255;
            }
            if(characteristics.is_joining_vertex_counterpart) {
                g = 255;
            }
            if(characteristics.is_leading_to_joining_vertex) {
                b = 128;
            }
            if(characteristics.is_combined_by_joining_vertex) {
                b = 255;
            }
        }
    } else {
        state->getColor(r, g, b);
    }

    QString style = parent_ ? parent_->styleSheet() : styleSheet();

    if(r >= 0 && g >= 0 && b >= 0) {
        QColor background(r,g,b);

        bool light = (background.lightness() > 128);

        QColor border = light ? background.darker(160) : background.lighter(160);
        QColor background_selected = light ? background.darker(140) : background.lighter(140);
        QColor border_selected = light ? border.darker(140) : border.lighter(140);
        text_color = light ? Qt::black: Qt::white;

        style += "csapex--NodeBox QFrame#boxframe { ";
        style += "background-color: rgb(" + QString::number(background.red()) + ", " +
                QString::number(background.green()) + ", " +
                QString::number(background.blue()) + ");";
        style += "border-color: rgb(" + QString::number(border.red()) + ", " +
                QString::number(border.green()) + ", " +
                QString::number(border.blue()) + ");";
        style += "}";
        style += "csapex--NodeBox[focused=\"true\"] QFrame#boxframe { ";
        style += "background-color: rgb(" + QString::number(background.red()) + ", " +
                QString::number(background_selected.green()) + ", " +
                QString::number(background_selected.blue()) + ");";
        style += "border-color: rgb(" + QString::number(border.red()) + ", " +
                QString::number(border_selected.green()) + ", " +
                QString::number(border_selected.blue()) + ");";
        style += "}";
    }

    style += "csapex--NodeBox QLabel, csapex--NodeBox QGroupBox { ";
    style += "color: rgb(" + QString::number(text_color.red()) + ", " +
            QString::number(text_color.green()) + ", " +
            QString::number(text_color.blue()) + ") !important;";
    style += "}";

    setStyleSheet(style);
}

void NodeBox::changeColor()
{
    updateStylesheetColor();
    refreshTopLevelStylesheet();
}

void NodeBox::updateVisuals()
{
    if(!ui || !ui->boxframe) {
        return;
    }

    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return;
    }
    NodeStatePtr state = nh->getNodeState();

    bool flip = state->isFlipped();

    setProperty("flipped", flip);

    if(ui && ui->boxframe) {
        if(grip_) {
            auto* layout = dynamic_cast<QGridLayout*>(ui->boxframe->layout());
            if(layout) {
                if(flip) {
                    layout->addWidget(grip_, 3, 0, Qt::AlignBottom | Qt::AlignLeft);
                } else {
                    layout->addWidget(grip_, 3, 2, Qt::AlignBottom | Qt::AlignRight);
                }
            }
        }

        ui->boxframe->setLayoutDirection(flip ? Qt::RightToLeft : Qt::LeftToRight);
        ui->frame->setLayoutDirection(Qt::LeftToRight);

        bool flag_set = ui->boxframe->property("content_minimized").toBool();
        bool minimized = isMinimizedSize();

        if(minimized != flag_set) {
            ui->boxframe->setProperty("content_minimized", minimized);

            if(minimized) {
                ui->frame->hide();
                info_exec->hide();
                ui->input_panel->hide();
                ui->output_panel->hide();
                ui->slot_panel->hide();
                ui->event_panel->hide();

                if(grip_) {
                    grip_->hide();
                }

                ui->gridLayout->removeWidget(ui->enablebtn);
                ui->gridLayout->addWidget(ui->enablebtn, 2, 0);

                ui->header_spacer->changeSize(0, 0);

            } else {
                ui->header_spacer->changeSize(0, 0, QSizePolicy::Expanding, QSizePolicy::Expanding);
                ui->header_spacer->invalidate();

                ui->gridLayout->removeWidget(ui->enablebtn);
                ui->gridLayout->addWidget(ui->enablebtn, 1, 0);

                ui->frame->show();
                info_exec->show();
                ui->output_panel->show();
                ui->input_panel->show();
                ui->output_panel->show();
                ui->slot_panel->show();
                ui->event_panel->show();

                if(grip_) {
                    grip_->show();
                }

            }
            layout()->invalidate();
        }
    }

    refreshTopLevelStylesheet();

    ensurePolished();
    adjustSize();
}

void NodeBox::updatePosition()
{
    auto pt = getNodeHandle()->getNodeState()->getPos();
    move(QPoint(pt.x, pt.y));
}

bool NodeBox::isMinimizedSize() const
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return false;
    }
    NodeStatePtr state = nh->getNodeState();
    return state->isMinimized();
}

bool NodeBox::isFlipped() const
{
    NodeHandlePtr nh = node_handle_.lock();
    if(!nh) {
        return false;
    }
    NodeStatePtr state = nh->getNodeState();
    return state->isFlipped();
}

bool NodeBox::hasSubGraph() const
{
    return dynamic_cast<Graph*>(getNode()) != nullptr;
}

GraphFacade* NodeBox::getSubGraph() const
{
    NodeHandlePtr nh = node_handle_.lock();
    if(nh) {
        NodePtr node = nh->getNode().lock();
        if(node) {
            return parent_->getGraphFacade()->getSubGraph(node->getUUID());
        }
    }

    throw std::logic_error("Called getSubGraph() on an invalid node. "
                           "Check with hasSubGraph().");
}

void NodeBox::nodeStateChangedEvent()
{
    NodeWorkerPtr worker = node_worker_.lock();
    if(!worker) {
        return;
    }
    NodeStatePtr state = worker->getNodeHandle()->getNodeState();

    bool state_enabled = state->isEnabled();
    bool box_enabled = !property("disabled").toBool();
    if(state_enabled != box_enabled) {
        ui->label->setEnabled(state_enabled);
        enabledChange(state_enabled);
    }

    setLabel(state->getLabel());
    ui->label->setToolTip(QString::fromStdString(worker->getUUID().getFullName()));

    updateThreadInformation();

    updateVisuals();
    updatePosition();
}
/// MOC
#include "../../../include/csapex/view/node/moc_box.cpp"
