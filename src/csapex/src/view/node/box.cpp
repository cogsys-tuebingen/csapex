/// HEADER
#include <csapex/view/node/box.h>

/// COMPONENT
#include <csapex/core/settings.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/node_state.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/node/node_adapter.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/widgets/meta_port.h>
#include <csapex/view/widgets/port.h>

#include "ui_box.h"

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

const std::chrono::milliseconds NodeBox::cache_update_rate_(500);

NodeBox::NodeBox(Settings& settings, NodeFacadePtr node_facade, QIcon icon, GraphView* parent)
    : parent_(parent), ui(nullptr), grip_(nullptr), settings_(settings), node_facade_(node_facade), adapter_(nullptr), icon_(icon),
      info_exec(nullptr), info_compo(nullptr), info_thread(nullptr), info_frequency(nullptr), info_error(nullptr), initialized_(false),
      frequency_timer_(nullptr),
      last_state_request_(std::chrono::milliseconds(0))
{
    QObject::connect(this, &NodeBox::updateVisualsRequest, this, &NodeBox::updateVisuals);

    setVisible(false);
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

    adapter_.reset();

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

    if(node_facade_->isVariadic()) {
        AUUID parent = node_facade_->getUUID().getAbsoluteUUID();
        if(node_facade_->hasVariadicInputs()) {
            MetaPort* meta_port = new MetaPort(ConnectorType::INPUT, parent);
            QObject::connect(meta_port, &MetaPort::createPortRequest, this, &NodeBox::createPortRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &NodeBox::createPortAndConnectRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &NodeBox::createPortAndMoveRequest);
            ui->input_panel->layout()->addWidget(meta_port);
        }
        if(node_facade_->hasVariadicOutputs()) {
            MetaPort* meta_port = new MetaPort(ConnectorType::OUTPUT, parent);
            QObject::connect(meta_port, &MetaPort::createPortRequest, this, &NodeBox::createPortRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &NodeBox::createPortAndConnectRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &NodeBox::createPortAndMoveRequest);
            ui->output_panel->layout()->addWidget(meta_port);
        }
        if(node_facade_->hasVariadicSlots()) {
            MetaPort* meta_port = new MetaPort(ConnectorType::SLOT_T, parent);
            QObject::connect(meta_port, &MetaPort::createPortRequest, this, &NodeBox::createPortRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &NodeBox::createPortAndConnectRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &NodeBox::createPortAndMoveRequest);
            ui->slot_panel->layout()->addWidget(meta_port);
        }
        if(node_facade_->hasVariadicEvents()) {
            MetaPort* meta_port = new MetaPort(ConnectorType::EVENT, parent);
            QObject::connect(meta_port, &MetaPort::createPortRequest, this, &NodeBox::createPortRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndConnectRequest, this, &NodeBox::createPortAndConnectRequest);
            QObject::connect(meta_port, &MetaPort::createPortAndMoveRequest, this, &NodeBox::createPortAndMoveRequest);
            ui->event_panel->layout()->addWidget(meta_port);
        }
    }


    NodeState* state = node_facade_->getNodeState().get();
    observer_.observeQueued(state->flipped_changed, this, &NodeBox::triggerFlipSides);
    observer_.observeQueued(state->minimized_changed, this, &NodeBox::triggerMinimized);
    observer_.observeQueued(state->enabled_changed, this, &NodeBox::triggerEnabledChanged);
    observer_.observeQueued(state->active_changed, [this, state](){
        setProperty("active", state->isActive());
        updateVisualsRequest();
    });
    setProperty("active", state->isActive());

    observer_.observeQueued(state->color_changed, this, &NodeBox::changeColor);
    observer_.observeQueued(state->pos_changed, this, &NodeBox::updatePosition);

    observer_.observeQueued(settings_.setting_changed, [this](const std::string& name) {
        if(name == "debug") {
            changeColor();
        }
    });

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
}

void NodeBox::construct()
{
    ui = new Ui::Box;
    ui->setupUi(this);

    ui->input_layout->addSpacerItem(new QSpacerItem(16, 0));
    ui->output_layout->addSpacerItem(new QSpacerItem(16, 0));

    ui->enablebtn->setCheckable(true);
    ui->enablebtn->setChecked(node_facade_->getNodeState()->isEnabled());

    QSize size(16, 16);
    ui->icon->setPixmap(icon_.pixmap(size));

    setFocusPolicy(Qt::ClickFocus);

    const UUID& uuid = node_facade_->getUUID();
    setToolTip(QString::fromStdString(uuid.getFullName()));

    setObjectName(QString::fromStdString(uuid.getFullName()));

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    if(grip_) {
        grip_->installEventFilter(this);
    }

    setLabel(node_facade_->getLabel());

    QObject::connect(ui->enablebtn, &QCheckBox::toggled, this, &NodeBox::toggled);


    observer_.observeQueued(node_facade_->node_state_changed, [this](NodeStatePtr state) {
        nodeStateChanged();
    });
    QObject::connect(this, &NodeBox::nodeStateChanged, this, &NodeBox::nodeStateChangedEvent, Qt::QueuedConnection);

    observer_.observeQueued(node_facade_->destroyed, [this](){ destruct(); });

    enabledChangeEvent(node_facade_->isProcessingEnabled());

    QObject::connect(this, &NodeBox::enabledChange, this, &NodeBox::enabledChangeEvent, Qt::QueuedConnection);

    setupUi();

    installEventFilter(this);
}

bool NodeBox::isGraph() const
{
    return node_facade_->isGraph();
}

NodeFacadePtr NodeBox::getNodeFacade() const
{
    return node_facade_;
}

NodeAdapter::Ptr NodeBox::getNodeAdapter() const
{
    return adapter_;
}

GraphView* NodeBox::getGraphView() const
{
    return parent_;
}


void NodeBox::updateBoxInformation(GraphFacade* graph)
{
    updateComponentInformation(graph);
    updateThreadInformation();
    updateFrequencyInformation();
}

void NodeBox::setStyleForId(QLabel* label, int id)
{
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


    if(settings_.getTemporary("debug", false)) {
        r = b = g = 128;
        if(node_facade_->canStartStepping()) {
            g = 255;
        } else {
            r = 255;
        }
    }
    ss << "QLabel { background-color : rgb(" << r << "," << g << "," << b << "); color: rgb(" << fr << "," << fg << "," << fb << ");}";
    label->setStyleSheet(ss.str().c_str());
}

void NodeBox::updateComponentInformation(GraphFacade* graph)
{
    if(settings_.getTemporary("debug", false)) {
        changeColor();
    }

    if(!settings_.getPersistent<bool>("display-graph-components", false)) {
        info_compo->setVisible(false);
        return;
    } else {
        info_compo->setVisible(true);
    }

    if(info_compo->isVisible()) {
        int compo = graph->getComponent(node_facade_->getUUID());
        int depth = graph->getDepth(node_facade_->getUUID());
        std::stringstream info;
        info << "C:" << compo;
        info << "D:" << depth;
        info_compo->setText(info.str().c_str());

        const NodeCharacteristics& chara = node_facade_->getNodeCharacteristics();

        QString tooltip("characteristics: ");;
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
    if(!settings_.getPersistent("display-threads", false)) {
        info_thread->setVisible(false);
        return;
    } else {
        info_thread->setVisible(true);
    }

    if(info_thread->isVisible()) {
        NodeStatePtr state = node_facade_->getNodeState();
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
    if(!settings_.getPersistent("display-frequencies", false)) {
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
        double max_f = node_facade_->getMaximumFrequency();
        double f = node_facade_->getExecutionFrequency();
        std::stringstream info;
        info << "<i><b>";
        info << std::setprecision(4) << f;
        if(max_f > 0.0) {
            info << " / " << max_f;
        }
        info << "Hz</b></i>";
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
    return node_facade_->isError();
}
ErrorState::ErrorLevel NodeBox::errorLevel() const
{
    return node_facade_->errorLevel();
}
std::string NodeBox::errorMessage() const
{
    return node_facade_->errorMessage();
}

void NodeBox::setLabel(const std::string& label)
{
    NodeStatePtr state = node_facade_->getNodeState();

    apex_assert_hard(state);
    state->setLabel(label);
    ui->label->setText(label.c_str());
    ui->label->setToolTip(label.c_str());
}

void NodeBox::setLabel(const QString &label)
{
    NodeStatePtr state = node_facade_->getNodeState();
    state->setLabel(label.toStdString());
    ui->label->setText(label);
}

std::string NodeBox::getLabel() const
{
    NodeStatePtr state = node_facade_->getNodeState();
    return state->getLabel();
}

void NodeBox::resizeEvent(QResizeEvent */*e*/)
{
}

void NodeBox::init()
{
    NodeStatePtr state = node_facade_->getNodeState();
    updatePosition();
    (*state->pos_changed)();

    setVisible(true);
}

Port* NodeBox::createPort(ConnectorPtr connector, QBoxLayout *layout)
{
    apex_assert_hard(QApplication::instance()->thread() == QThread::currentThread());

    Port* port = new Port(connector);

    port->setFlipped(isFlipped());
    port->setMinimizedSize(isMinimizedSize());

    QObject::connect(this, SIGNAL(minimized(bool)), port, SLOT(setMinimizedSize(bool)));
    QObject::connect(this, SIGNAL(flipped(bool)), port, SLOT(setFlipped(bool)));

    ConnectorPtr adaptee = port->getAdaptee();
    apex_assert_hard(adaptee == connector);

    if(node_facade_->isVariadic()) {
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

void NodeBox::removePort(ConnectorWeakPtr connector)
{
    ConnectorPtr adaptee = connector.lock();
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
            if(isGraph()) {
                Q_EMIT showSubGraphRequest(node_facade_->getSubgraphAUUID());
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
    QString state;

    auto now = std::chrono::high_resolution_clock::now();
    if(now - last_state_request_ > cache_update_rate_) {
        cached_state_ = node_facade_->getExecutionState();
        last_state_request_ = now;
    }

    switch(cached_state_) {
    case ExecutionState::IDLE:
        state = "idle"; break;
    case ExecutionState::ENABLED:
        state = "enabled"; break;
    case ExecutionState::FIRED:
        state = "fired"; break;
    case ExecutionState::PROCESSING:
        state = "processing"; break;
    default:
        state = "unknown"; break;
    }

    return state;
}

void NodeBox::paintEvent(QPaintEvent* /*e*/)
{
    if(!adapter_) {
        return;
    }
    QString state = getNodeState();
    QString transition_state = settings_.getTemporary("debug", false)
            ? QString::fromStdString(node_facade_->getDebugDescription()) : QString();

    QString state_text;
    state_text = "<img src=\":/node_";
    state_text += state + ".png\" />";
    if(node_facade_->getNodeState()->isMuted()) {
        state_text += "<img src=\":/muted.png\" />";
    }


    info_exec->setText(state_text);
    info_exec->setToolTip(state + transition_state);

    bool is_error = node_facade_->isError() && node_facade_->errorLevel() == ErrorState::ErrorLevel::ERROR;
    bool is_warn = node_facade_->isError() && node_facade_->errorLevel() == ErrorState::ErrorLevel::WARNING;


    bool error_change = ui->boxframe->property("error").toBool() != is_error;
    bool warning_change = ui->boxframe->property("warning").toBool() != is_warn;

    setProperty("error", is_error);
    setProperty("warning", is_warn);

    if(error_change || warning_change) {
        if(is_error) {
            QString msg = QString::fromStdString(node_facade_->errorMessage());
            setToolTip(msg);
            info_error->setToolTip(msg);
            info_error->setVisible(true);
        } else if(is_warn) {
            QString msg = QString::fromStdString(node_facade_->errorMessage());
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
    if(adapter_) {
        adapter_->stop();
    }
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
    node_facade_->setProfiling(profiling);
}

void NodeBox::triggerFlipSides()
{
    NodeStatePtr state = node_facade_->getNodeState();
    bool flip = state->isFlipped();
    Q_EMIT flipped(flip);
}

void NodeBox::triggerMinimized()
{
    NodeStatePtr state = node_facade_->getNodeState();
    bool minimize = state->isMinimized();
    Q_EMIT minimized(minimize);
}

void NodeBox::triggerEnabledChanged()
{
    NodeStatePtr state = node_facade_->getNodeState();

    bool state_enabled = state->isEnabled();
    bool box_enabled = !property("disabled").toBool();
    if(state_enabled != box_enabled) {
        ui->label->setEnabled(state_enabled);
        enabledChange(state_enabled);
    }
}

void NodeBox::updateStylesheetColor()
{
    NodeStatePtr state = node_facade_->getNodeState();

    QColor text_color = Qt::black;

    int r, g, b;
    if(settings_.getTemporary("debug", false)) {
        r = 0; g = 0; b = 0;
        const NodeCharacteristics& characteristics = node_facade_->getNodeCharacteristics();
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

    NodeStatePtr state = node_facade_->getNodeState();

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
    auto pt = getNodeFacade()->getNodeState()->getPos();
    move(QPoint(pt.x, pt.y));
}

bool NodeBox::isMinimizedSize() const
{
    NodeStatePtr state = node_facade_->getNodeState();
    return state->isMinimized();
}

bool NodeBox::isFlipped() const
{
    NodeStatePtr state = node_facade_->getNodeState();
    return state->isFlipped();
}

void NodeBox::nodeStateChangedEvent()
{
    triggerEnabledChanged();

    NodeStatePtr state = node_facade_->getNodeState();
    setLabel(state->getLabel());
    ui->label->setToolTip(QString::fromStdString(node_facade_->getUUID().getFullName()));

    updateThreadInformation();

    updateVisuals();
    updatePosition();
}
/// MOC
#include "../../../include/csapex/view/node/moc_box.cpp"
