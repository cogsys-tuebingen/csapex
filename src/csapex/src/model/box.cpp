/// HEADER
#include <csapex/model/box.h>

/// COMPONENT
#include "ui_box.h"
#include <csapex/model/node.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/node_worker.h>
#include <csapex/command/move_box.h>
#include <csapex/command/delete_box.h>
#include <csapex/command/meta.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/profiling_widget.h>

/// SYSTEM
#include <QDragMoveEvent>
#include <QGraphicsSceneDragDropEvent>
#include <QInputDialog>
#include <QMenu>
#include <QThread>
#include <QTimer>
#include <iostream>
#include <boost/foreach.hpp>
#include <cmath>

using namespace csapex;

const QString Box::MIME = "csapex/model/box";
const QString Box::MIME_MOVE = "csapex/model/box/move";

const Box::Ptr Box::NullPtr;

void Box::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Flow;
    out << YAML::BeginMap;
    if(parent) {
        out << YAML::Key << "type";
        out << YAML::Value << parent->node_->getType();
    }
    out << YAML::Key << "uuid";
    out << YAML::Value << uuid_;
    out << YAML::Key << "label";
    out << YAML::Value << label_;
    out << YAML::Key << "pos";
    out << YAML::Value << YAML::BeginSeq << pos.x() << pos.y() << YAML::EndSeq;
    out << YAML::Key << "minimized";
    out << YAML::Value << minimized;
    out << YAML::Key << "enabled";
    out << YAML::Value << enabled;

    if(parent) {
        boxed_state = parent->node_->getState();
    }

    if(boxed_state.get()) {
        out << YAML::Key << "state";
        out << YAML::Value << YAML::BeginMap;
        boxed_state->writeYaml(out);
        out << YAML::EndMap;
    }

    out << YAML::EndMap;
}

void Box::State::copyFrom(const Box::State::Ptr& rhs)
{
    operator =(*rhs);
    boxed_state = parent->node_->getState();
    if(rhs->boxed_state) {
        *boxed_state = *rhs->boxed_state;
    }
}

void Box::State::readYaml(const YAML::Node &node)
{
    if(node.FindValue("minimized")) {
        node["minimized"] >> minimized;
    }

    if(node.FindValue("enabled")) {
        node["enabled"] >> enabled;
    }

    if(node.FindValue("label")) {
        node["label"] >> label_;
    }

    if(node.FindValue("state")) {
        const YAML::Node& state_map = node["state"];
        boxed_state = parent->node_->getState();
        boxed_state->readYaml(state_map);
    }
}


Box::Box(Node::Ptr node, NodeAdapter::Ptr adapter, const std::string& uuid, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), dispatcher_(NULL), node_(node), adapter_(adapter), state(new State(this)),
      private_thread_(NULL), worker_(new NodeWorker(node)), down_(false), profiling_(false)
{
    ui->setupUi(this);

    node_->setBox(this);

    ui->enablebtn->setCheckable(node_->canBeDisabled());

    setFocusPolicy(Qt::ClickFocus);

    state->uuid_ = uuid;
    setToolTip(uuid.c_str());

    setObjectName(uuid.c_str());

    setLabel(uuid);

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    ui->enablebtn->setIcon(node->getIcon());

    state->minimized = false;

    QObject::connect(this, SIGNAL(tickRequest()), worker_, SLOT(tick()));
    QObject::connect(this, SIGNAL(toggled(bool)), node_.get(), SIGNAL(toggled(bool)));
    QObject::connect(this, SIGNAL(placed()), node_.get(), SIGNAL(started()));

    QObject::connect(worker_, SIGNAL(messageProcessed()), this, SLOT(messageProcessed()));

    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));
    QObject::connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SLOT(enableContent(bool)));

    QObject::connect(node_.get(), SIGNAL(modelChanged()), this, SLOT(eventModelChanged()));
    QObject::connect(node_.get(), SIGNAL(connectorCreated(Connector*)), this, SLOT(registerEvent(Connector*)));
    QObject::connect(node_.get(), SIGNAL(connectorRemoved(Connector*)), this, SLOT(unregisterEvent(Connector*)));
    QObject::connect(&adapter_->bridge, SIGNAL(guiChanged()), worker_, SLOT(eventGuiChanged()), Qt::QueuedConnection);

    setContextMenuPolicy(Qt::CustomContextMenu);

    QObject::connect(this, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(showContextMenu(QPoint)));

    setVisible(false);


    adapter_->fill(ui->content);
}

NodePtr Box::getNode()
{
    return node_;
}

void Box::messageProcessed()
{
    foreach(ConnectorIn* i, node_->input) {
        i->notify();
    }
}

void Box::makeThread()
{
    if(!private_thread_) {
        private_thread_ = new QThread;
        connect(private_thread_, SIGNAL(finished()), private_thread_, SLOT(deleteLater()));

        assert(worker_);
        worker_->moveToThread(private_thread_);

        private_thread_->start();
    }
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

    state->enabled = enable;

    node_->enable(enable);
    ui->label->setEnabled(enable);
}

YAML::Emitter& Box::save(YAML::Emitter& out) const
{
    state->writeYaml(out);

    return out;
}

void Box::read(YAML::Node &doc)
{
    Memento::Ptr s = getState();
    s->readYaml(doc);
    setState(s);
}

void Box::stop()
{
    foreach(ConnectorIn* i, node_->input) {
        disconnectConnector(i);
    }
    foreach(ConnectorOut* i, node_->output) {
        disconnectConnector(i);
    }

    QObject::disconnect(private_thread_);
    QObject::disconnect(worker_);
    QObject::disconnect(this);

    if(private_thread_) {
        private_thread_->quit();
        private_thread_->wait(1000);
        if(private_thread_->isRunning()) {
            std::cout << "terminate thread" << std::endl;
            private_thread_->terminate();
        }
    }
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
    return state->uuid_;
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
    state->label_ = label;
    ui->label->setText(label.c_str());
    ui->label->setToolTip(label.c_str());
}

void Box::setLabel(const QString &label)
{
    state->label_ = label.toStdString();
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
    if(c->isOutput()) {
        removeOutputEvent(dynamic_cast<ConnectorOut*>(c));
    } else {
        removeInputEvent(dynamic_cast<ConnectorIn*>(c));
    }
}

void Box::registerInputEvent(ConnectorIn* in)
{
    worker_->addInput(in);

    in->setParent(NULL);
    ui->input_layout->addWidget(in);

    QObject::connect(in, SIGNAL(messageArrived(Connector*)), worker_, SLOT(forwardMessage(Connector*)));

    connectConnector(in);

    Q_EMIT changed(this);
}

void Box::registerOutputEvent(ConnectorOut* out)
{
    assert(out);

    out->setParent(NULL);
    assert(ui);
    assert(ui->output_layout);
    ui->output_layout->addWidget(out);

    connectConnector(out);

    Q_EMIT changed(this);
}

int Box::nextInputId()
{
    return node_->input.size();
}

int Box::nextOutputId()
{
    return node_->output.size();
}

void Box::removeInputEvent(ConnectorIn *in)
{
    disconnectConnector(in);
}

void Box::removeOutputEvent(ConnectorOut *out)
{
    disconnectConnector(out);
}

void Box::connectConnector(Connector *c)
{
    QObject::connect(c, SIGNAL(connectionFormed(Connector*,Connector*)), this, SIGNAL(connectionFormed(Connector*,Connector*)));
    QObject::connect(c, SIGNAL(connectionDestroyed(Connector*,Connector*)), this, SIGNAL(connectionDestroyed(Connector*,Connector*)));
    QObject::connect(c, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::connect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    QObject::connect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
}


void Box::disconnectConnector(Connector *c)
{
    QObject::disconnect(c, SIGNAL(connectionFormed(Connector*,Connector*)), this, SIGNAL(connectionFormed(Connector*,Connector*)));
    QObject::disconnect(c, SIGNAL(connectionDestroyed(Connector*,Connector*)), this, SIGNAL(connectionDestroyed(Connector*,Connector*)));
    QObject::disconnect(c, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::disconnect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    QObject::disconnect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
}

void Box::resizeEvent(QResizeEvent *)
{
    Q_EMIT changed(this);
}


int Box::countInputs()
{
    return node_->input.size();
}

int Box::countOutputs()
{
    return node_->output.size();
}

void Box::init(const QPoint& pos)
{
    if(parent()) {
        setVisible(true);
    }

    move(pos);
    key_point = pos;

    makeThread();
}

Box::~Box()
{
    stop();

    // TODO: remove from graph!!!!

    delete worker_;
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

    state->pos = e->pos();

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
    dispatcher_->execute(dispatcher_->getGraph()->moveSelectedBoxes(delta));
}

void Box::deleteBox()
{
    dispatcher_->execute(Command::Ptr(new command::DeleteBox(UUID())));
}

void Box::refreshStylesheet()
{
    ui->boxframe->style()->unpolish(ui->boxframe);
    ui->boxframe->style()->polish(ui->boxframe);
}

void Box::eventModelChanged()
{
    adapter_->updateDynamicGui(ui->content);
}

void Box::tick()
{
    if(state->enabled) {
        Q_EMIT tickRequest();
    }
}

NodeWorker* Box::getNodeWorker()
{
    return worker_;
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
    if(private_thread_ && private_thread_->isRunning()) {

        QMutexLocker lock(&worker_mutex_);

        QObject::disconnect(private_thread_);
        QObject::disconnect(worker_);

        QObject::connect(private_thread_, SIGNAL(finished()), private_thread_, SLOT(deleteLater()));
        QObject::connect(private_thread_, SIGNAL(terminated()), private_thread_, SLOT(deleteLater()));

        QObject::connect(private_thread_, SIGNAL(finished()), worker_, SLOT(deleteLater()));
        QObject::connect(private_thread_, SIGNAL(terminated()), worker_, SLOT(deleteLater()));

        private_thread_->quit();
        if(!private_thread_->wait(100)) {
            private_thread_->terminate();
        }

        private_thread_ = NULL;
        worker_ = new NodeWorker(node_);

        QObject::connect(this, SIGNAL(tickRequest()), worker_, SLOT(tick()));
        BOOST_FOREACH(ConnectorIn* in, node_->input) {
            QObject::connect(in, SIGNAL(messageArrived(ConnectorIn*)), worker_, SLOT(forwardMessage(ConnectorIn*)));
        }

        makeThread();
    }
}

bool Box::isMinimizedSize() const
{
    return state->minimized;
}

void Box::setSynchronizedInputs(bool sync)
{
    worker_->setSynchronizedInputs(sync);
}

void Box::minimizeBox(bool minimize)
{    
    state->minimized = minimize;

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

Memento::Ptr Box::getState() const
{
    assert(state);

    State::Ptr memento(new State);
    *memento = *state;

    memento->boxed_state = node_->getState();

    return memento;
}

void Box::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    std::string old_uuid = state->uuid_;
    std::string old_label = state->label_;

    *state = *m;

    if(state->label_.empty()) {
        state->label_ = old_label;
    }
    if(state->uuid_.empty()) {
        state->uuid_ = old_uuid;
    }

    state->parent = this;
    if(m->boxed_state != NULL) {
        node_->setState(m->boxed_state);
    }

    minimizeBox(state->minimized);

    enableContent(state->enabled);
    ui->enablebtn->setChecked(state->enabled);

    setLabel(state->label_);
    ui->label->setToolTip(state->uuid_.c_str());
}

Command::Ptr Box::removeAllConnectionsCmd()
{
    command::Meta::Ptr cmd(new command::Meta);

    BOOST_FOREACH(ConnectorIn* i, node_->input) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }
    BOOST_FOREACH(ConnectorOut* i, node_->output) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }

    return cmd;
}

CommandDispatcher* Box::getCommandDispatcher() const
{
    return dispatcher_;
}

void Box::setCommandDispatcher(CommandDispatcher *d)
{
    dispatcher_ = d;
}
