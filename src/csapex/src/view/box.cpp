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

const QString Box::MIME = "csapex/model/box";
const QString Box::MIME_MOVE = "csapex/model/box/move";

Box::Box(NodePtr node, NodeAdapter::Ptr adapter, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), node_(node), adapter_(adapter),
      down_(false), info_compo(NULL), profiling_(false), is_placed_(false)
{
}

Box::~Box()
{
}


void Box::setupUi()
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

void Box::setupUiAgain()
{
    adapter_->doSetupUi(ui->content);
    updateFlippedSides();
}

void Box::construct()
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

    setContextMenuPolicy(Qt::CustomContextMenu);

    QObject::connect(this, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(showContextMenu(QPoint)));

    for(int i = 0; i < node_->countInputs(); ++i) {
        registerInputEvent(node_->getInput(i));
    }
    for(int i = 0; i < node_->countOutputs(); ++i) {
        registerOutputEvent(node_->getOutput(i));
    }

    setupUi();
}

Node* Box::getNode()
{
    return node_.get();
}

void Box::enableContent(bool enable)
{
    node_->enable(enable);

    ui->label->setEnabled(enable);
}

#define _HSV2RGB_(H, S, V, R, G, B) \
{ \
  double _h = H/60.; \
  int _hf = (int)floor(_h); \
  int _hi = ((int)_h)%6; \
  double _f = _h - _hf; \
  \
  double _p = V * (1. - S); \
  double _q = V * (1. - _f * S); \
  double _t = V * (1. - (1. - _f) * S); \
  \
  switch (_hi) \
  { \
    case 0: \
        R = 255.*V; G = 255.*_t; B = 255.*_p; \
    break; \
    case 1: \
        R = 255.*_q; G = 255.*V; B = 255.*_p; \
    break; \
    case 2: \
        R = 255.*_p; G = 255.*V; B = 255.*_t; \
    break; \
    case 3: \
        R = 255.*_p; G = 255.*_q; B = 255.*V; \
    break; \
    case 4: \
        R = 255.*_t; G = 255.*_p; B = 255.*V; \
    break; \
    case 5: \
        R = 255.*V; G = 255.*_p; B = 255.*_q; \
    break; \
  } \
}

void Box::updateInformation(Graph* graph)
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
    _HSV2RGB_(hue, 1., 1., r, g, b);
    double fr = 0, fb = 0, fg = 0;
    if(b > 100 && r < 100 && g < 100) {
        fr = fb = fg = 255;
    }
    std::stringstream ss;
    ss << "QLabel { background-color : rgb(" << r << "," << g << "," << b << "); color: rgb(" << fr << "," << fg << "," << fb << ");}";
    info_compo->setStyleSheet(ss.str().c_str());
}

void Box::showContextMenu(const QPoint& pos)
{
    Q_EMIT showContextMenuForBox(this, mapToGlobal(pos));
}

void Box::fillContextMenu(QMenu *menu, std::map<QAction*, boost::function<void()> >& handler)
{
    ContextMenuHandler::addHeader(*menu, std::string("Node: ") + node_->getUUID().getShortName());

    if(isMinimizedSize()) {
        QAction* max = new QAction("maximize", menu);
        max->setIcon(QIcon(":/maximize.png"));
        max->setIconVisibleInMenu(true);
        handler[max] = boost::bind(&Box::minimizeBox, this, false);
        menu->addAction(max);

    } else {
        QAction* min = new QAction("minimize", menu);
        min->setIcon(QIcon(":/minimize.png"));
        min->setIconVisibleInMenu(true);
        handler[min] = boost::bind(&Box::minimizeBox, this, true);
        menu->addAction(min);
    }

    QAction* flip = new QAction("flip sides", menu);
    flip->setIcon(QIcon(":/flip.png"));
    flip->setIconVisibleInMenu(true);
    handler[flip] = boost::bind(&Box::flipSides, this);
    menu->addAction(flip);

    menu->addSeparator();

    QAction* term = new QAction("terminate thread", menu);
    term->setIcon(QIcon(":/stop.png"));
    term->setIconVisibleInMenu(true);
    handler[term] = boost::bind(&Box::killContent, this);
    menu->addAction(term);

    QAction* prof = new QAction("profiling", menu);
    prof->setIcon(QIcon(":/profiling.png"));
    prof->setIconVisibleInMenu(true);
    handler[prof] = boost::bind(&Box::showProfiling, this);
    menu->addAction(prof);

    menu->addSeparator();

    QAction* del = new QAction("delete", menu);
    del->setIcon(QIcon(":/close.png"));
    del->setIconVisibleInMenu(true);
    handler[del] = boost::bind(&Box::deleteBox, this);
    menu->addAction(del);
}

QBoxLayout* Box::getInputLayout()
{
    return ui->input_layout;
}

QBoxLayout* Box::getOutputLayout()
{
    return ui->output_layout;
}

bool Box::isError() const
{
    return node_->isError();
}
ErrorState::ErrorLevel Box::errorLevel() const
{
    return node_->errorLevel();
}
std::string Box::errorMessage() const
{
    return node_->errorMessage();
}

void Box::setError(bool e, const std::string &msg)
{
    setError(e, msg, ErrorState::EL_ERROR);
}

void Box::setError(bool, const std::string &msg, int)
{
    setToolTip(msg.c_str());
    //node_->setErrorSilent(e, msg, level);
}

void Box::setLabel(const std::string& label)
{
    assert(node_->getNodeState());
    node_->setLabel(label);
    ui->label->setText(label.c_str());
    ui->label->setToolTip(label.c_str());
}

void Box::setLabel(const QString &label)
{
    node_->setLabel(label.toStdString());
    ui->label->setText(label);
}

std::string Box::getLabel() const
{
    return node_->getLabel();
}

void Box::registerEvent(Connectable* c)
{
    if(c->isOutput()) {
        registerOutputEvent(dynamic_cast<ConnectorOut*>(c));
    } else {
        registerInputEvent(dynamic_cast<ConnectorIn*>(c));
    }
}

void Box::unregisterEvent(Connectable*)
{
}

void Box::registerInputEvent(ConnectorIn* in)
{
    in->setParent(NULL);

    Q_EMIT changed(this);
}

void Box::registerOutputEvent(ConnectorOut* out)
{
    assert(out);

    out->setParent(NULL);

    Q_EMIT changed(this);
}

void Box::resizeEvent(QResizeEvent *)
{
    Q_EMIT changed(this);
}

void Box::init()
{
    if(parent()) {
        setVisible(true);
    } else {
        setVisible(false);
    }

    key_point = node_->getNodeState()->pos;
    move(key_point);
}
bool Box::eventFilter(QObject* o, QEvent* e)
{
    QMouseEvent* em = dynamic_cast<QMouseEvent*>(e);

    if(o == ui->label) {
        if(e->type() == QEvent::MouseButtonDblClick && em->button() == Qt::LeftButton) {
            bool ok;
            QString text = QInputDialog::getText(this, "Box Label", "Enter new name", QLineEdit::Normal, getLabel().c_str(), &ok);

            if(ok && !text.isEmpty()) {
                setLabel(text);
            }

            e->accept();

            return true;
        }
    }

    if(o == ui->content || o == ui->label || o == this) {
        if(e->type() == QEvent::MouseButtonPress && em->button() == Qt::LeftButton) {
            down_ = true;
            start_drag_global_ = em->globalPos();
            start_drag_ = em->pos();

        } else if(e->type() == QEvent::MouseButtonRelease && em->button() == Qt::LeftButton) {
            down_ = false;
            Q_EMIT clicked(this);
            e->accept();
            return true;

        } else if(e->type() == QEvent::MouseMove) {
            QPoint delta = em->globalPos() - start_drag_global_;

            bool shift_drag = Qt::ShiftModifier == QApplication::keyboardModifiers();

            if(down_) {
                if(shift_drag) {
                    if(hypot(delta.x(), delta.y()) > 15) {
                        //BoxManager::instance().startPlacingBox(parentWidget(), node_->getType(), -start_drag_);
                        down_ = false;
                    }
                } else {
                    e->ignore();

                    startDrag(-start_drag_);

                    down_ = false;
                    return true;
                }
            }
        }
    }

    //    if(e->type() == QEvent::MouseButtonRelease && em->button() == Qt::RightButton && !isSelected()) {
    //        Q_EMIT clicked(this);
    //        Q_EMIT showContextMenuForBox(this, em->globalPos());
    //    }

    return false;
}

void Box::enabledChange(bool val)
{
    ui->boxframe->setProperty("disabled", !val);

    refreshStylesheet();
}

void Box::paintEvent(QPaintEvent*)
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

void Box::mousePressEvent(QMouseEvent* e)
{
    eventFilter(this, e);
}
void Box::mouseReleaseEvent(QMouseEvent* e)
{
    eventFilter(this, e);
}
void Box::mouseMoveEvent(QMouseEvent* e)
{
    eventFilter(this, e);
}

void Box::moveEvent(QMoveEvent* e)
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

void Box::triggerPlaced()
{
    Q_EMIT placed();
}

void Box::selectEvent()
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

void Box::deselectEvent()
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

void Box::keyPressEvent(QKeyEvent *)
{

}

void Box::stop()
{
    QObject::disconnect(this);
    adapter_->stop();
}

void Box::startDrag(QPoint offset)
{
    QDrag* drag = new QDrag(this);
    QMimeData* mimeData = new QMimeData;
    mimeData->setText(node_->getUUID().c_str());

    QByteArray ox;
    QByteArray oy;

    ox.setNum(offset.x());
    oy.setNum(offset.y());

    mimeData->setData(Box::MIME_MOVE, QByteArray());
    mimeData->setData(Box::MIME_MOVE + "/x", ox);
    mimeData->setData(Box::MIME_MOVE + "/y", oy);

    drag->setMimeData(mimeData);

    lower();

    if(Qt::ShiftModifier == QApplication::keyboardModifiers()) {
        //BoxManager::instance().startPlacingBox(parentWidget(), node_->getType(), offset);
        return;
    }

    if(!isSelected()) {
        Q_EMIT clicked(this);
    }

    QPoint start_pos = pos();
    /*Qt::DropAction action = */drag->exec();

    QPoint end_pos = pos();

    QPoint delta = end_pos - start_pos;
    // TODO: ugly

    Q_EMIT moveRequest(this,delta);
    //node_->getCommandDispatcher()->execute(node_->getCommandDispatcher()->getGraph()->moveSelectedBoxes(delta));
}

void Box::deleteBox()
{
    node_->getCommandDispatcher()->execute(Command::Ptr(new command::DeleteNode(node_->getUUID())));
}

void Box::refreshStylesheet()
{
    setStyleSheet(styleSheet());
}

void Box::eventModelChanged()
{
    setupUi();

    adapter_->updateDynamicGui(ui->content);
}

void Box::showProfiling()
{
    profiling_ = !profiling_;

    if(profiling_) {
        prof = new ProfilingWidget(parentWidget(), this);
        prof->show();
    } else {
        delete prof;
    }

}

void Box::killContent()
{
    node_->killContent();
}

void Box::flipSides()
{
    bool& flipped = node_->node_state_->flipped;
    flipped = !flipped;
    updateFlippedSides();
}

void Box::updateFlippedSides()
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

bool Box::isMinimizedSize() const
{
    return node_->getNodeState()->minimized;
}

void Box::minimizeBox(bool minimize)
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

bool Box::hasSubGraph()
{
    return false;
}

Graph::Ptr Box::getSubGraph()
{
    throw std::runtime_error("cannot call getSubGraph() on Box! Check with hasSubGraph()!");
}

void Box::nodeStateChanged()
{
    minimizeBox(node_->getNodeState()->minimized);

    enableContent(node_->getNodeState()->enabled);
    ui->enablebtn->setChecked(node_->getNodeState()->enabled);

    setLabel(node_->getLabel());
    ui->label->setToolTip(node_->getUUID().c_str());

    move(node_->getPosition());
}

CommandDispatcher* Box::getCommandDispatcher() const
{
    return node_->getCommandDispatcher();
}

void Box::setCommandDispatcher(CommandDispatcher *d)
{
    node_->setCommandDispatcher(d);
}
