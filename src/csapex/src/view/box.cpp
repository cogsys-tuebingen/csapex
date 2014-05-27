/// HEADER
#include <csapex/view/box.h>

/// COMPONENT
#include "ui_box.h"
#include <csapex/model/node.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/meta.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/profiling_widget.h>
#include <csapex/view/node_adapter.h>
#include <csapex/view/port.h>
#include <csapex/utility/context_menu_handler.h>
#include <csapex/utility/color.hpp>

/// SYSTEM
#include <QDragMoveEvent>
#include <QGraphicsSceneDragDropEvent>
#include <QInputDialog>
#include <QMenu>
#include <QTimer>
#include <iostream>
#include <boost/foreach.hpp>
#include <cmath>

using namespace csapex;

const QString NodeBox::MIME = "csapex/model/box";
const QString NodeBox::MIME_MOVE = "csapex/model/box/move";

NodeBox::NodeBox(NodePtr node, NodeAdapter::Ptr adapter, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), node_(node), adapter_(adapter),
      down_(false), info_compo(NULL), profiling_(false), is_placed_(false)
{
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

    ui->enablebtn->setCheckable(node_->canBeDisabled());

    setFocusPolicy(Qt::ClickFocus);

    const UUID& uuid = node_->getUUID();
    setToolTip(uuid.c_str());

    setObjectName(uuid.c_str());

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    setLabel(node_->getLabel());

    ui->enablebtn->setIcon(node_->getIcon());

    node_->setMinimized(false);

    QObject::connect(this, SIGNAL(toggled(bool)), node_.get(), SIGNAL(toggled(bool)));
    QObject::connect(this, SIGNAL(placed()), node_.get(), SIGNAL(started()));

    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));
    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SLOT(enableContent(bool)));

    QObject::connect(node_.get(), SIGNAL(destroyed()), this, SLOT(deleteLater()));
    QObject::connect(node_.get(), SIGNAL(modelChanged()), this, SLOT(eventModelChanged()));
    QObject::connect(node_.get(), SIGNAL(connectorCreated(Connectable*)), this, SLOT(registerEvent(Connectable*)));
    QObject::connect(node_.get(), SIGNAL(connectorRemoved(Connectable*)), this, SLOT(unregisterEvent(Connectable*)));
    QObject::connect(node_.get(), SIGNAL(stateChanged()), this, SLOT(nodeStateChanged()));
    QObject::connect(node_.get(), SIGNAL(enabled(bool)), this, SLOT(enabledChange(bool)));
    QObject::connect(node_.get(), SIGNAL(nodeError(bool,std::string,int)), this, SLOT(setError(bool, std::string, int)));

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
    node_->enable(enable);

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
    double r, g, b;
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

    QAction* prof = new QAction("profiling", menu);
    prof->setIcon(QIcon(":/profiling.png"));
    prof->setIconVisibleInMenu(true);
    handler[prof] = boost::bind(&NodeBox::showProfiling, this);
    menu->addAction(prof);

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
    assert(node_->getNodeState());
    node_->setLabel(label);
    ui->label->setText(label.c_str());
    ui->label->setToolTip(label.c_str());
}

void NodeBox::setLabel(const QString &label)
{
    node_->setLabel(label.toStdString());
    ui->label->setText(label);
}

std::string NodeBox::getLabel() const
{
    return node_->getLabel();
}

void NodeBox::registerEvent(Connectable* c)
{
    if(c->isOutput()) {
        registerOutputEvent(dynamic_cast<ConnectorOut*>(c));
    } else {
        registerInputEvent(dynamic_cast<ConnectorIn*>(c));
    }
}

void NodeBox::unregisterEvent(Connectable*)
{
}

void NodeBox::registerInputEvent(ConnectorIn* in)
{
    in->setParent(NULL);

    Q_EMIT changed(this);
}

void NodeBox::registerOutputEvent(ConnectorOut* out)
{
    assert(out);

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

    move(node_->getNodeState()->pos);
}
bool NodeBox::eventFilter(QObject* /*o*/, QEvent* /*e*/)
{
//    QMouseEvent* em = dynamic_cast<QMouseEvent*>(e);

//    if(o == ui->label) {
//        if(e->type() == QEvent::MouseButtonDblClick && em->button() == Qt::LeftButton) {
//            bool ok;
//            QString text = QInputDialog::getText(this, "Box Label", "Enter new name", QLineEdit::Normal, getLabel().c_str(), &ok);

//            if(ok && !text.isEmpty()) {
//                setLabel(text);
//            }

//            e->accept();

//            return true;
//        }
//    }

//    if(o == ui->content || o == ui->label || o == this) {
//        if(e->type() == QEvent::MouseButtonPress && em->button() == Qt::LeftButton) {
//            down_ = true;
//            start_drag_global_ = em->globalPos();
//            start_drag_ = em->pos();

//        } else if(e->type() == QEvent::MouseButtonRelease && em->button() == Qt::LeftButton) {
//            down_ = false;
//            Q_EMIT clicked(this);
//            e->accept();
//            return true;

//        } else if(e->type() == QEvent::MouseMove) {
//            QPoint delta = em->globalPos() - start_drag_global_;

//            bool shift_drag = Qt::ShiftModifier == QApplication::keyboardModifiers();

//            if(down_) {
//                if(shift_drag) {
//                    if(hypot(delta.x(), delta.y()) > 15) {
//                        //BoxManager::instance().startPlacingBox(parentWidget(), node_->getType(), -start_drag_);
//                        down_ = false;
//                    }
//                } else {
//                    e->ignore();

//                    startDrag(-start_drag_);

//                    down_ = false;
//                    return true;
//                }
//            }
//        }
//    }

    //    if(e->type() == QEvent::MouseButtonRelease && em->button() == Qt::RightButton && !isSelected()) {
    //        Q_EMIT clicked(this);
    //        Q_EMIT showContextMenuForBox(this, em->globalPos());
    //    }

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

//void Box::mousePressEvent(QMouseEvent* e)
//{
//    eventFilter(this, e);
//}
//void Box::mouseReleaseEvent(QMouseEvent* e)
//{
//    eventFilter(this, e);
//}
//void Box::mouseMoveEvent(QMouseEvent* e)
//{
//    eventFilter(this, e);
//}

void NodeBox::moveEvent(QMoveEvent* e)
{
    if(!is_placed_) {
        is_placed_ = true;
        return;
    }

    eventFilter(this, e);

    QPoint delta = e->pos() - e->oldPos();

    node_->setPosition(e->pos());

    Q_EMIT moved(this, delta.x(), delta.y());
}

void NodeBox::triggerPlaced()
{
    Q_EMIT placed();
}

void NodeBox::selectEvent()
{
    //#error TODO: make this into a signal, connect port via signal!
    //    Q_FOREACH(ConnectorIn* i, node_->getInputs()){
    //        Port* p = i->getPort();
    //        if(p) {
    //            p->setSelected(true);
    //        }
    //    }
    //    Q_FOREACH(ConnectorOut* i, node_->getOutputs()) {
    //        Port* p = i->getPort();
    //        if(p) {
    //            p->setSelected(true);
    //        }
    //    }

    ui->boxframe->setProperty("focused",true);
    refreshStylesheet();
}

void NodeBox::deselectEvent()
{
    //    Q_FOREACH(ConnectorIn* i, node_->getInputs()){
    //        Port* p = i->getPort();
    //        if(p) {
    //            p->setSelected(false);
    //        }
    //    }
    //    Q_FOREACH(ConnectorOut* i, node_->getOutputs()) {
    //        Port* p = i->getPort();
    //        if(p) {
    //            p->setSelected(false);
    //        }
    //    }
    ui->boxframe->setProperty("focused",false);
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
    node_->getCommandDispatcher()->execute(Command::Ptr(new command::DeleteNode(node_->getUUID())));
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
        prof = new ProfilingWidget(parentWidget(), this);
        prof->show();
    } else {
        delete prof;
    }

}

void NodeBox::killContent()
{
    node_->killContent();
}

void NodeBox::flipSides()
{
    bool& flipped = node_->node_state_->flipped;
    flipped = !flipped;
    updateFlippedSides();
}

void NodeBox::updateFlippedSides()
{
    const bool& flip = node_->node_state_->flipped;

    ui->boxframe->setLayoutDirection(flip ? Qt::RightToLeft : Qt::LeftToRight);
    ui->frame->setLayoutDirection(Qt::LeftToRight);

    Q_EMIT flipped(flip);
    //    BOOST_FOREACH(ConnectorIn* i, node_->getInputs()){
    //        Port* p = i->getPort();
    //        if(p) {
    //            p->setFlipped(flip);
    //        }
    //    }
    //    BOOST_FOREACH(ConnectorOut* i, node_->getOutputs()) {
    //        Port* p = i->getPort();
    //        if(p) {
    //            p->setFlipped(flip);
    //        }
    //    }
}

bool NodeBox::isMinimizedSize() const
{
    return node_->getNodeState()->minimized;
}

void NodeBox::minimizeBox(bool minimize)
{    
    node_->setMinimized(minimize);

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
    minimizeBox(node_->getNodeState()->minimized);

    enableContent(node_->getNodeState()->enabled);
    ui->enablebtn->setChecked(node_->getNodeState()->enabled);

    setLabel(node_->getLabel());
    ui->label->setToolTip(node_->getUUID().c_str());

    move(node_->getPosition());
}

CommandDispatcher* NodeBox::getCommandDispatcher() const
{
    return node_->getCommandDispatcher();
}

void NodeBox::setCommandDispatcher(CommandDispatcher *d)
{
    node_->setCommandDispatcher(d);
}
