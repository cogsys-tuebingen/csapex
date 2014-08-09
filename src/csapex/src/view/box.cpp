/// HEADER
#include <csapex/view/box.h>

/// COMPONENT
#include "ui_box.h"
#include <csapex/model/node.h>
#include <csapex/manager/box_manager.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/meta.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/node_adapter.h>
#include <csapex/view/port.h>
#include <csapex/utility/context_menu_handler.h>
#include <csapex/utility/color.hpp>
#include <csapex/core/settings.h>

/// SYSTEM
#include <QDragMoveEvent>
#include <QGraphicsSceneDragDropEvent>
#include <QMenu>
#include <QTimer>
#include <iostream>
#include <boost/foreach.hpp>
#include <cmath>

using namespace csapex;

const QString NodeBox::MIME = "csapex/model/box";

NodeBox::NodeBox(Settings& settings, CommandDispatcher* cmd_dispatcher, NodePtr node, NodeAdapter::Ptr adapter, QIcon icon, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), settings_(settings), cmd_dispatcher_(cmd_dispatcher), node_(node), adapter_(adapter), icon_(icon),
      down_(false), info_compo(NULL), profiling_(false), is_placed_(false)
{
    node_->setCommandDispatcher(cmd_dispatcher);
}

NodeBox::~NodeBox()
{
}


void NodeBox::setupUi()
{
    node_->checkConditions(true);

    if(!info_compo) {
        info_compo = new QLabel;
        info_compo->setProperty("component", true);
        ui->infos->addWidget(info_compo);
    }

    QObject::connect(node_->getNodeWorker(), SIGNAL(messagesReceived()), this, SLOT(setupUiAgain()));
    adapter_->doSetupUi(ui->content);

    updateFlippedSides();

    Q_EMIT changed(this);
}

void NodeBox::setupUiAgain()
{
    adapter_->doSetupUi(ui->content);
    updateFlippedSides();
}

void NodeBox::construct()
{
    ui->setupUi(this);

    ui->input_layout->addSpacerItem(new QSpacerItem(16, 0));
    ui->output_layout->addSpacerItem(new QSpacerItem(16, 0));

    ui->enablebtn->setCheckable(true);

    ui->enablebtn->setIcon(icon_);

    setFocusPolicy(Qt::ClickFocus);

    const UUID& uuid = node_->getUUID();
    setToolTip(uuid.c_str());

    setObjectName(uuid.c_str());

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    setLabel(node_->getNodeState()->getLabel());

    node_->getNodeWorker()->setMinimized(false);

    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SLOT(enableContent(bool)));

    QObject::connect(node_.get(), SIGNAL(destroyed()), this, SLOT(deleteLater()));
    QObject::connect(node_.get(), SIGNAL(modelChanged()), this, SLOT(eventModelChanged()));
    QObject::connect(node_.get(), SIGNAL(connectorCreated(Connectable*)), this, SLOT(registerEvent(Connectable*)));
    QObject::connect(node_.get(), SIGNAL(connectorRemoved(Connectable*)), this, SLOT(unregisterEvent(Connectable*)));
    QObject::connect(node_.get(), SIGNAL(stateChanged()), this, SLOT(nodeStateChanged()));
    QObject::connect(node_.get(), SIGNAL(nodeError(bool,std::string,int)), this, SLOT(setError(bool, std::string, int)));

    NodeWorker* worker_ = node_->getNodeWorker();
    QObject::connect(worker_, SIGNAL(enabled(bool)), this, SLOT(enabledChange(bool)));

    for(int i = 0; i < node_->countInputs(); ++i) {
        registerInputEvent(node_->getInput(i));
    }
    for(int i = 0; i < node_->countOutputs(); ++i) {
        registerOutputEvent(node_->getOutput(i));
    }

    setupUi();
}

Node* NodeBox::getNode()
{
    return node_.get();
}

NodeAdapter::Ptr NodeBox::getNodeAdapter()
{
    return adapter_;
}

void NodeBox::enableContent(bool enable)
{
    node_->getNodeWorker()->setEnabled(enable);

    ui->label->setEnabled(enable);
}

void NodeBox::updateInformation(Graph* graph)
{
    int compo = graph->getComponent(node_->getUUID());
    if(compo < 0) {
        return;
    }

    std::stringstream info;
    info << compo;
    info_compo->setText(info.str().c_str());

    // set color using HSV rotation
    double hue =  (compo * 77) % 360;
    double r = 0, g = 0, b = 0;
    __HSV2RGB__(hue, 1., 1., r, g, b);
    double fr = 0, fb = 0, fg = 0;
    if(b > 100 && r < 100 && g < 100) {
        fr = fb = fg = 255;
    }
    std::stringstream ss;
    ss << "QLabel { background-color : rgb(" << r << "," << g << "," << b << "); color: rgb(" << fr << "," << fg << "," << fb << ");}";
    info_compo->setStyleSheet(ss.str().c_str());
}

void NodeBox::contextMenuEvent(QContextMenuEvent* e)
{
    Q_EMIT showContextMenuForBox(this, e->globalPos());
}

void NodeBox::fillContextMenu(QMenu *menu, std::map<QAction*, boost::function<void()> >& handler)
{
    ContextMenuHandler::addHeader(*menu, std::string("Node: ") + node_->getUUID().getShortName());

    if(isMinimizedSize()) {
        QAction* max = new QAction("maximize", menu);
        max->setIcon(QIcon(":/maximize.png"));
        max->setIconVisibleInMenu(true);
        handler[max] = boost::bind(&NodeBox::minimizeBox, this, false);
        menu->addAction(max);

    } else {
        QAction* min = new QAction("minimize", menu);
        min->setIcon(QIcon(":/minimize.png"));
        min->setIconVisibleInMenu(true);
        handler[min] = boost::bind(&NodeBox::minimizeBox, this, true);
        menu->addAction(min);
    }

    QAction* flip = new QAction("flip sides", menu);
    flip->setIcon(QIcon(":/flip.png"));
    flip->setIconVisibleInMenu(true);
    handler[flip] = boost::bind(&NodeBox::flipSides, this);
    menu->addAction(flip);

    menu->addSeparator();

    QAction* term = new QAction("terminate thread", menu);
    term->setIcon(QIcon(":/stop.png"));
    term->setIconVisibleInMenu(true);
    handler[term] = boost::bind(&NodeBox::killContent, this);
    menu->addAction(term);

    QAction* prof;
    if(profiling_) {
        prof = new QAction("stop profiling", menu);
        prof->setIcon(QIcon(":/stop_profiling.png"));
    } else {
        prof = new QAction("profiling", menu);
        prof->setIcon(QIcon(":/profiling.png"));
    }
    prof->setIconVisibleInMenu(true);
    handler[prof] = boost::bind(&NodeBox::showProfiling, this);
    menu->addAction(prof);

    QAction* info = new QAction("get information", menu);
    info->setIcon(QIcon(":/help.png"));
    info->setIconVisibleInMenu(true);
    handler[info] = boost::bind(&NodeBox::getInformation, this);
    menu->addAction(info);

    menu->addSeparator();

    QAction* del = new QAction("delete", menu);
    del->setIcon(QIcon(":/close.png"));
    del->setIconVisibleInMenu(true);
    handler[del] = boost::bind(&NodeBox::deleteBox, this);
    menu->addAction(del);
}

QBoxLayout* NodeBox::getInputLayout()
{
    return ui->input_layout;
}

QBoxLayout* NodeBox::getOutputLayout()
{
    return ui->output_layout;
}

bool NodeBox::isError() const
{
    return node_->isError();
}
ErrorState::ErrorLevel NodeBox::errorLevel() const
{
    return node_->errorLevel();
}
std::string NodeBox::errorMessage() const
{
    return node_->errorMessage();
}

void NodeBox::setError(bool e, const std::string &msg)
{
    setError(e, msg, ErrorState::EL_ERROR);
}

void NodeBox::setError(bool, const std::string &msg, int)
{
    setToolTip(msg.c_str());
    //node_->setErrorSilent(e, msg, level);
}

void NodeBox::setLabel(const std::string& label)
{
    apex_assert_hard(node_->getNodeState());
    node_->getNodeState()->setLabel(label);
    ui->label->setText(label.c_str());
    ui->label->setToolTip(label.c_str());
}

void NodeBox::setLabel(const QString &label)
{
    node_->getNodeState()->setLabel(label.toStdString());
    ui->label->setText(label);
}

std::string NodeBox::getLabel() const
{
    return node_->getNodeState()->getLabel();
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

void NodeBox::registerInputEvent(Input* in)
{
    in->setParent(NULL);

    Q_EMIT changed(this);
}

void NodeBox::registerOutputEvent(Output* out)
{
    apex_assert_hard(out);

    out->setParent(NULL);

    Q_EMIT changed(this);
}

void NodeBox::resizeEvent(QResizeEvent *)
{
    Q_EMIT changed(this);
}

void NodeBox::init()
{
    if(parent()) {
        setVisible(true);
    } else {
        setVisible(false);
    }

    move(node_->getNodeState()->getPos());
}

bool NodeBox::eventFilter(QObject* o, QEvent* e)
{
    QMouseEvent* em = dynamic_cast<QMouseEvent*>(e);

    if(o == ui->label) {
        if(e->type() == QEvent::MouseButtonDblClick && em->button() == Qt::LeftButton) {
            Q_EMIT renameRequest(this);
            e->accept();

            return true;
        }
    }

    return false;
}

void NodeBox::enabledChange(bool val)
{
    ui->boxframe->setProperty("disabled", !val);

    refreshStylesheet();
}

void NodeBox::paintEvent(QPaintEvent*)
{
    if(!node_ || !adapter_) {
        return;
    }

    bool is_error = node_->isError() && node_->errorLevel() == ErrorState::EL_ERROR;
    bool is_warn = node_->isError() && node_->errorLevel() == ErrorState::EL_WARNING;

    bool error_change = ui->boxframe->property("error").toBool() != is_error;
    bool warning_change = ui->boxframe->property("warning").toBool() != is_warn;

    ui->boxframe->setProperty("error", is_error);
    ui->boxframe->setProperty("warning", is_warn);

    if(error_change || warning_change) {
        if(is_error) {
            ui->label->setToolTip(node_->errorMessage().c_str());
        } else if(is_warn) {
            ui->label->setToolTip(node_->errorMessage().c_str());
        } else {
            ui->label->setToolTip(node_->getUUID().c_str());
        }

        refreshStylesheet();
    }

    resize(sizeHint());
}

void NodeBox::moveEvent(QMoveEvent* e)
{
    if(!is_placed_) {
        is_placed_ = true;
        return;
    }

    eventFilter(this, e);


    QPoint pos = e->pos();
    if(settings_.get("grid-lock", false)) {
        pos.setX(round(pos.x() / 10.0) * 10.0);
        pos.setY(round(pos.y() / 10.0) * 10.0);
        move(pos);
    }

    QPoint delta = pos - e->oldPos();

    node_->getNodeState()->setPos(pos);

    Q_EMIT moved(this, delta.x(), delta.y());
}

void NodeBox::triggerPlaced()
{
    Q_EMIT placed();
}

void NodeBox::setSelected(bool selected)
{
    ui->boxframe->setProperty("focused",selected);
    refreshStylesheet();
}

void NodeBox::keyPressEvent(QKeyEvent *)
{

}

void NodeBox::stop()
{
    QObject::disconnect(this);
    adapter_->stop();
}

void NodeBox::deleteBox()
{
    cmd_dispatcher_->execute(Command::Ptr(new command::DeleteNode(node_->getUUID())));
}

void NodeBox::getInformation()
{
    Q_EMIT helpRequest(this);
}

void NodeBox::refreshStylesheet()
{
    setStyleSheet(styleSheet());
}

void NodeBox::eventModelChanged()
{
    setupUi();

    adapter_->updateDynamicGui(ui->content);
}

void NodeBox::showProfiling()
{
    profiling_ = !profiling_;

    if(profiling_) {
        Q_EMIT profile(this);
    } else {
        Q_EMIT stopProfiling(this);
    }
}

void NodeBox::killContent()
{
    node_->getNodeWorker()->killExecution();
}

void NodeBox::flipSides()
{
    node_->getNodeState()->setFlipped(!node_->getNodeState()->isFlipped());
    updateFlippedSides();
}

void NodeBox::updateFlippedSides()
{
    bool flip = node_->getNodeState()->isFlipped();

    ui->boxframe->setLayoutDirection(flip ? Qt::RightToLeft : Qt::LeftToRight);
    ui->frame->setLayoutDirection(Qt::LeftToRight);

    Q_EMIT flipped(flip);
}

bool NodeBox::isMinimizedSize() const
{
    return node_->getNodeState()->isMinimized();
}

bool NodeBox::isFlipped() const
{
    return node_->getNodeState()->isFlipped();
}

void NodeBox::minimizeBox(bool minimize)
{    
    node_->getNodeWorker()->setMinimized(minimize);

    Q_EMIT minimized(minimize);

    if(minimize) {
        ui->frame->hide();
        ui->label->hide();
        ui->boxframe->setProperty("content_minimized", true);

    } else {
        ui->frame->show();
        ui->label->show();
        ui->boxframe->setProperty("content_minimized", false);
    }

    refreshStylesheet();

    resize(sizeHint());
}

bool NodeBox::hasSubGraph()
{
    return false;
}

Graph::Ptr NodeBox::getSubGraph()
{
    throw std::runtime_error("cannot call getSubGraph() on Box! Check with hasSubGraph()!");
}

void NodeBox::nodeStateChanged()
{
    minimizeBox(node_->getNodeState()->isMinimized());

    enableContent(node_->getNodeState()->isEnabled());
    ui->enablebtn->setChecked(node_->getNodeState()->isEnabled());

    setLabel(node_->getNodeState()->getLabel());
    ui->label->setToolTip(node_->getUUID().c_str());

    move(node_->getNodeState()->getPos());
}
