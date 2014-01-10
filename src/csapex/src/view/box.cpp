/// HEADER
#include <csapex/view/box.h>

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
#include <csapex/view/port.h>

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

int Box::g_instances = 0;

Box::Box(Node* node, NodeAdapter::Ptr adapter, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), node_(node), adapter_(adapter),
      down_(false), profiling_(false), is_placed_(false)
{
    adapter_->setNode(node_);

    construct(node);

    ++g_instances;
}

Box::Box(BoxedObject* node, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), node_(node), adapter_shared_(node),
      down_(false), profiling_(false), is_placed_(false)
{
    construct(node);

    ++g_instances;
}

Box::~Box()
{
    --g_instances;
}


void Box::setupUi()
{
    if(adapter_) {
        QObject::connect(&adapter_->bridge, SIGNAL(guiChanged()), node_, SLOT(eventGuiChanged()), Qt::QueuedConnection);
        adapter_->doSetupUi(ui->content);
    } else {
        QObject::connect(&adapter_shared_->bridge, SIGNAL(guiChanged()), node_, SLOT(eventGuiChanged()), Qt::QueuedConnection);
        adapter_shared_->doSetupUi(ui->content);
    }
}

void Box::construct(Node* node)
{
    ui->setupUi(this);

    ui->input_layout->addSpacerItem(new QSpacerItem(16, 0));
    ui->output_layout->addSpacerItem(new QSpacerItem(16, 0));

    node_->setBox(this);

    ui->enablebtn->setCheckable(node_->canBeDisabled());

    setFocusPolicy(Qt::ClickFocus);

    const std::string& uuid = node_->getUUID();
    setToolTip(uuid.c_str());

    setObjectName(uuid.c_str());

    if(getLabel().empty()) {
        setLabel(uuid);
    }

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    ui->enablebtn->setIcon(node->getIcon());

    node_->setMinimized(false);

    QObject::connect(this, SIGNAL(toggled(bool)), node_, SIGNAL(toggled(bool)));
    QObject::connect(this, SIGNAL(placed()), node_, SIGNAL(started()));

    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));
    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SLOT(enableContent(bool)));

    QObject::connect(node_, SIGNAL(destroyed()), this, SLOT(deleteLater()));
    QObject::connect(node_, SIGNAL(modelChanged()), this, SLOT(eventModelChanged()));
    QObject::connect(node_, SIGNAL(connectorCreated(Connectable*)), this, SLOT(registerEvent(Connectable*)));
    QObject::connect(node_, SIGNAL(connectorRemoved(Connectable*)), this, SLOT(unregisterEvent(Connectable*)));
    QObject::connect(node_, SIGNAL(stateChanged()), this, SLOT(nodeStateChanged()));

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
    return node_;
}

void Box::enableContent(bool enable)
{
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
    node_->setErrorSilent(e, msg, level);
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
    return ui->label->text().toStdString();
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
    ui->input_layout->addWidget(new Port(getCommandDispatcher(), in));

    Q_EMIT changed(this);
}

void Box::registerOutputEvent(ConnectorOut* out)
{
    assert(out);

    out->setParent(NULL);
    assert(ui);
    assert(ui->output_layout);
    ui->output_layout->addWidget(new Port(getCommandDispatcher(), out));

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
                        BoxManager::instance().startPlacingBox(parentWidget(), node_->getType(), -start_drag_);
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
    BOOST_FOREACH(ConnectorIn* i, node_->input){
        i->getPort()->setSelected(true);
    }
    BOOST_FOREACH(ConnectorOut* i, node_->output) {
        i->getPort()->setSelected(true);
    }
    ui->boxframe->setProperty("focused",true);
    refreshStylesheet();
}

void Box::deselectEvent()
{
    BOOST_FOREACH(ConnectorIn* i, node_->input){
        i->getPort()->setSelected(false);
    }
    BOOST_FOREACH(ConnectorOut* i, node_->output) {
        i->getPort()->setSelected(false);
    }
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
        BoxManager::instance().startPlacingBox(parentWidget(), node_->getType(), offset);
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
    node_->getCommandDispatcher()->execute(Command::Ptr(new command::DeleteNode(node_->getUUID())));
}

void Box::refreshStylesheet()
{
    setStyleSheet(styleSheet());
}

void Box::eventModelChanged()
{
    //    if(is_updating_gui_) {
    //        return;
    //    }

    setupUi();

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
    node_->setMinimized(minimize);

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
