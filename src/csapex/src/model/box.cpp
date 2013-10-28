/// HEADER
#include <csapex/model/box.h>

/// COMPONENT
#include "ui_box.h"
#include <csapex/model/node.h>
#include <csapex/model/boxed_object.h>
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

Box::Box(Node* node, NodeAdapter::Ptr adapter, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), node_(node), adapter_(adapter),
      down_(false), profiling_(false)
{
    construct(node);
}

Box::Box(BoxedObject* node, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), node_(node), adapter_shared_(node),
      down_(false), profiling_(false)
{
    construct(node);
}

void Box::construct(Node* node)
{
    ui->setupUi(this);

    node_->setBox(this);

    ui->enablebtn->setCheckable(node_->canBeDisabled());

    setFocusPolicy(Qt::ClickFocus);

    const std::string& uuid = node_->uuid_;
    setToolTip(uuid.c_str());

    setObjectName(uuid.c_str());

    setLabel(uuid);

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    ui->enablebtn->setIcon(node->getIcon());

    node_->getNodeState()->minimized = false;

    QObject::connect(this, SIGNAL(toggled(bool)), node_, SIGNAL(toggled(bool)));
    QObject::connect(this, SIGNAL(placed()), node_, SIGNAL(started()));

    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));
    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SLOT(enableContent(bool)));

    QObject::connect(node_, SIGNAL(destroyed()), this, SLOT(deleteLater()));
    QObject::connect(node_, SIGNAL(modelChanged()), this, SLOT(eventModelChanged()));
    QObject::connect(node_, SIGNAL(connectorCreated(Connector*)), this, SLOT(registerEvent(Connector*)));
    QObject::connect(node_, SIGNAL(connectorRemoved(Connector*)), this, SLOT(unregisterEvent(Connector*)));
    QObject::connect(node_, SIGNAL(stateChanged()), this, SLOT(nodeStateChanged()));

    setContextMenuPolicy(Qt::CustomContextMenu);

    QObject::connect(this, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(showContextMenu(QPoint)));


    if(adapter_) {
        QObject::connect(&adapter_->bridge, SIGNAL(guiChanged()), node_, SLOT(eventGuiChanged()), Qt::QueuedConnection);
        adapter_->fill(ui->content);
    } else {
        QObject::connect(&adapter_shared_->bridge, SIGNAL(guiChanged()), node_, SLOT(eventGuiChanged()), Qt::QueuedConnection);
        adapter_shared_->fill(ui->content);
    }
}

Node* Box::getNode()
{
    return node_;
}

void Box::enableIO(bool enable)
{
    foreach(ConnectorIn* i, node_->input) {
        i->setEnabled(enable);
    }
    foreach(ConnectorOut* i, node_->output) {
        i->setEnabled(enable);
    }
}

void Box::setIOError(bool error)
{
    foreach(ConnectorIn* i, node_->input) {
        i->setErrorSilent(error);
    }
    foreach(ConnectorOut* i, node_->output) {
        i->setErrorSilent(error);
    }
    enableIO(!error);
}

void Box::enableContent(bool enable)
{
    enableIO(enable);

    node_->getNodeState()->enabled = enable;

    node_->enable(enable);
    ui->label->setEnabled(enable);
}

void Box::stop()
{
    node_->stop();
}

void Box::showContextMenu(const QPoint& pos)
{
    Q_EMIT showContextMenuForBox(this, mapToGlobal(pos));
}

void Box::fillContextMenu(QMenu *menu, std::map<QAction*, boost::function<void()> >& handler)
{
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

std::string Box::UUID() const
{
    return node_->UUID();
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
void Box::setError(bool e, const std::string &msg, ErrorState::ErrorLevel level)
{
    setToolTip(msg.c_str());
    node_->setError(e, msg, level);
}

std::string Box::getType() const
{
    return node_->getType();
}

void Box::setLabel(const std::string& label)
{
    assert(node_->getNodeState());
    node_->getNodeState()->label_ = label;
    ui->label->setText(label.c_str());
    ui->label->setToolTip(label.c_str());
}

void Box::setLabel(const QString &label)
{
    node_->getNodeState()->label_ = label.toStdString();
    ui->label->setText(label);
}

std::string Box::getLabel() const
{
    return ui->label->text().toStdString();
}

void Box::registerEvent(Connector * c)
{
    if(c->isOutput()) {
        registerOutputEvent(dynamic_cast<ConnectorOut*>(c));
    } else {
        registerInputEvent(dynamic_cast<ConnectorIn*>(c));
    }
}

void Box::unregisterEvent(Connector * c)
{
}

void Box::registerInputEvent(ConnectorIn* in)
{
    in->setParent(NULL);
    ui->input_layout->addWidget(in);

    Q_EMIT changed(this);
}

void Box::registerOutputEvent(ConnectorOut* out)
{
    assert(out);

    out->setParent(NULL);
    assert(ui);
    assert(ui->output_layout);
    ui->output_layout->addWidget(out);

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

    node_->makeThread();
}

Box::~Box()
{
    stop();
}

bool Box::eventFilter(QObject* o, QEvent* e)
{
    QMouseEvent* em = dynamic_cast<QMouseEvent*>(e);

    if(o == ui->label) {
        if(e->type() == QEvent::MouseButtonDblClick && em->button() == Qt::LeftButton) {
            bool ok;
            QString text = QInputDialog::getText(this, "Box Label", "Enter new name", QLineEdit::Normal, getLabel().c_str(), &ok);

            if(ok) {
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
                        BoxManager::instance().startPlacingBox(parentWidget(), getType(), -start_drag_);
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
    if(val)  {
        node_->enable();
    } else {
        node_->disable();
    }
}

void Box::paintEvent(QPaintEvent*)
{
    bool is_error = node_->isError() && node_->errorLevel() == ErrorState::EL_ERROR;
    bool is_warn = node_->isError() && node_->errorLevel() == ErrorState::EL_WARNING;

    bool error_change = ui->boxframe->property("error").toBool() != is_error;
    bool warning_change = ui->boxframe->property("warning").toBool() != is_warn;

    ui->boxframe->setProperty("error", is_error);
    ui->boxframe->setProperty("warning", is_warn);

    if(error_change || warning_change) {
        if(is_error) {
            setLabel(QString("ERROR: ") + objectName());
            ui->label->setToolTip(node_->errorMessage().c_str());
        } else if(is_warn) {
            setLabel(QString("WARNING: ") + objectName());
            ui->label->setToolTip(node_->errorMessage().c_str());
        } else {
            setLabel(objectName());
            ui->label->setToolTip(UUID().c_str());
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
    eventFilter(this, e);

    QPoint delta = e->pos() - e->oldPos();

    node_->getNodeState()->pos = e->pos();

    Q_EMIT moved(this, delta.x(), delta.y());
}

void Box::triggerPlaced()
{
    Q_EMIT placed();
}

void Box::selectEvent()
{
    ui->boxframe->setProperty("focused",true);
    refreshStylesheet();
}

void Box::deselectEvent()
{
    ui->boxframe->setProperty("focused",false);
    refreshStylesheet();
}

void Box::keyPressEvent(QKeyEvent *)
{

}

void Box::startDrag(QPoint offset)
{
    QDrag* drag = new QDrag(this);
    QMimeData* mimeData = new QMimeData;
    mimeData->setText(UUID().c_str());

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
        BoxManager::instance().startPlacingBox(parentWidget(), getType(), offset);
        return;
    }

    if(!isSelected()) {
        Q_EMIT clicked(this);
    }

    QPoint start_pos = pos();
    /*Qt::DropAction action = */drag->exec();

    QPoint end_pos = pos();

    QPoint delta = end_pos - start_pos;
    node_->getCommandDispatcher()->execute(node_->getCommandDispatcher()->getGraph()->moveSelectedBoxes(delta));
}

void Box::deleteBox()
{
    node_->getCommandDispatcher()->execute(Command::Ptr(new command::DeleteNode(UUID())));
}

void Box::refreshStylesheet()
{
    ui->boxframe->style()->unpolish(ui->boxframe);
    ui->boxframe->style()->polish(ui->boxframe);
}

void Box::eventModelChanged()
{
    if(adapter_) {
        adapter_->updateDynamicGui(ui->content);
    } else {
        adapter_shared_->updateDynamicGui(ui->content);
    }
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

bool Box::isMinimizedSize() const
{
    return node_->getNodeState()->minimized;
}

void Box::minimizeBox(bool minimize)
{    
    node_->getNodeState()->minimized = minimize;

    BOOST_FOREACH(ConnectorIn* i, node_->input){
        i->setMinimizedSize(minimize);
    }
    BOOST_FOREACH(ConnectorOut* i, node_->output) {
        i->setMinimizedSize(minimize);
    }

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

    setLabel(node_->getNodeState()->label_);
    ui->label->setToolTip(UUID().c_str());
}

CommandDispatcher* Box::getCommandDispatcher() const
{
    return node_->getCommandDispatcher();
}

void Box::setCommandDispatcher(CommandDispatcher *d)
{
    node_->setCommandDispatcher(d);
}
