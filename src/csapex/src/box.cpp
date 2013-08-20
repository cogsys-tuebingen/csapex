/// HEADER
#include <csapex/box.h>

/// COMPONENT
#include "ui_box.h"
#include <csapex/boxed_object.h>
#include <csapex/box_manager.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/command_move_box.h>
#include <csapex/command_delete_box.h>
#include <csapex/command_meta.h>
#include <csapex/command_dispatcher.h>

/// SYSTEM
#include <QDragMoveEvent>
#include <QInputDialog>
#include <QMenu>
#include <QThread>
#include <QTimer>
#include <iostream>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>

using namespace csapex;

const QString Box::MIME = "csapex/box";
const QString Box::MIME_MOVE = "csapex/box/move";


void Box::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Flow;
    out << YAML::BeginMap;
    out << YAML::Key << "type";
    out << YAML::Value << type_;
    out << YAML::Key << "uuid";
    out << YAML::Value << uuid_;
    out << YAML::Key << "label";
    out << YAML::Value << label_;
    out << YAML::Key << "pos";
    out << YAML::Value << YAML::BeginSeq << parent->pos().x() << parent->pos().y() << YAML::EndSeq;
    out << YAML::Key << "minimized";
    out << YAML::Value << minimized;
    out << YAML::Key << "enabled";
    out << YAML::Value << enabled;

    boxed_state = parent->content_->getState();
    if(boxed_state.get()) {
        out << YAML::Key << "state";
        out << YAML::Value << YAML::BeginMap;
        boxed_state->writeYaml(out);
        out << YAML::EndMap;
    }

    out << YAML::EndMap;
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
        boxed_state = parent->content_->getState();
        boxed_state->readYaml(state_map);
    }
}


Box::Box(BoxedObject* content, const std::string& uuid, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), state(new State(this)), private_thread_(NULL), worker_(new BoxWorker(this)), down_(false), next_sub_id_(0)
{
    ui->setupUi(this);

    minimize_icon_ = ui->minimizebtn->icon();
    maximize_icon_ = QIcon(":/maximize.png");

    content_ = content;
    content_->setBox(this);

    ui->enablebtn->setCheckable(content_->canBeDisabled());

    setFocusPolicy(Qt::ClickFocus);

    state->uuid_ = uuid;
    setToolTip(uuid.c_str());
    ui->label->setToolTip(uuid.c_str());

    setObjectName(uuid.c_str());
    setLabel(uuid);

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    ui->enablebtn->setIcon(content->getIcon());

    state->minimized = false;

    QObject::connect(this, SIGNAL(tickRequest()), worker_, SLOT(tick()));

    connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));
    connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SLOT(enableContent(bool)));

    connect(ui->minimizebtn, SIGNAL(toggled(bool)), this, SLOT(minimizeBox(bool)));
    connect(ui->killbtn, SIGNAL(clicked()), this, SLOT(killContent()));

    connect(content, SIGNAL(modelChanged()), this, SLOT(eventModelChanged()), Qt::QueuedConnection);
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
    foreach(ConnectorIn* i, input) {
        i->setEnabled(enable);
    }
    foreach(ConnectorOut* i, output) {
        i->setEnabled(enable);
    }
}

void Box::setIOError(bool error)
{
    foreach(ConnectorIn* i, input) {
        i->setErrorSilent(error);
    }
    foreach(ConnectorOut* i, output) {
        i->setErrorSilent(error);
    }
    enableIO(!error);
}

void Box::enableContent(bool enable)
{
    enableIO(enable);

    state->enabled = enable;

    content_->enable(enable);
    ui->label->setEnabled(enable);
}

YAML::Emitter& Box::save(YAML::Emitter& out) const
{
    state->writeYaml(out);

    return out;
}

void Box::stop()
{
    foreach(ConnectorIn* i, input) {
        disconnectConnector(i);
    }
    foreach(ConnectorOut* i, output) {
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

void BoxWorker::forwardMessage(Connector *source)
{
    if(parent_->content_->isEnabled()) {
        parent_->content_->messageArrived(dynamic_cast<ConnectorIn*>(source));

        if(!parent_->content_->isError() || parent_->content_->errorLevel() != Displayable::EL_ERROR) {
            parent_->content_->setError(false);
        }
    }
}

void BoxWorker::tick()
{
    if(parent_->content_->isEnabled()) {
        parent_->content_->tick();
    }
}

Box* BoxWorker::parent()
{
    return parent_;
}

std::string Box::UUID() const
{
    return state->uuid_;
}

void Box::setType(const std::string& type)
{
    state->type_ = type;
}

std::string Box::getType() const
{
    return state->type_;
}

void Box::setLabel(const std::string& label)
{
    state->label_ = label;
    ui->label->setText(label.c_str());
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

void Box::addInput(ConnectorIn* in)
{
    in->setParent(NULL);
    ui->input_layout->addWidget(in);
    input.push_back(in);

    QObject::connect(in, SIGNAL(messageArrived(Connector*)), worker_, SLOT(forwardMessage(Connector*)));

    connectConnector(in);

    Q_EMIT connectorCreated(in);
    Q_EMIT changed(this);
}

void Box::addOutput(ConnectorOut* out)
{
    assert(out);
    out->setParent(NULL);
    assert(ui);
    assert(ui->output_layout);
    ui->output_layout->addWidget(out);
    output.push_back(out);

    QObject::connect(out, SIGNAL(connectionFormed(Connector*,Connector*)), this, SIGNAL(connectionFormed(Connector*,Connector*)));
    QObject::connect(out, SIGNAL(connectionDestroyed(Connector*,Connector*)), this, SIGNAL(connectionDestroyed(Connector*,Connector*)));

    connectConnector(out);

    Q_EMIT connectorCreated(out);
    Q_EMIT changed(this);
}

int Box::nextInputId()
{
    int nextId = next_sub_id_;

    next_sub_id_ = nextId + 1;

    return nextId;
}

int Box::nextOutputId()
{
    int nextId = next_sub_id_;

    next_sub_id_ = nextId + 1;

    return nextId;
}

void Box::removeInput(ConnectorIn *in)
{
    std::vector<ConnectorIn*>::iterator it;
    it = std::find(input.begin(), input.end(), in);

    assert(*it == in);

    disconnectConnector(in);

    in->deleteLater();
    input.erase(it);
}

void Box::removeOutput(ConnectorOut *out)
{
    std::vector<ConnectorOut*>::iterator it;
    it = std::find(output.begin(), output.end(), out);

    assert(*it == out);

    QObject::disconnect(out, SIGNAL(connectionFormed(ConnectorOut*,ConnectorIn*)), this, SIGNAL(connectionFormed(ConnectorOut*,ConnectorIn*)));
    QObject::disconnect(out, SIGNAL(connectionDestroyed(ConnectorOut*,ConnectorIn*)), this, SIGNAL(connectionDestroyed(ConnectorOut*,ConnectorIn*)));

    disconnectConnector(out);

    out->deleteLater();
    output.erase(it);
}

void Box::connectConnector(Connector *c)
{
    QObject::connect(c, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::connect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    QObject::connect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
    QObject::connect(c, SIGNAL(enabled(Connector*)), this, SIGNAL(connectorEnabled(Connector*)));
    QObject::connect(c, SIGNAL(disabled(Connector*)), this, SIGNAL(connectorDisabled(Connector*)));
}


void Box::disconnectConnector(Connector *c)
{
    QObject::disconnect(c, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::disconnect(c, SIGNAL(connectionStart()), this, SIGNAL(connectionStart()));
    QObject::disconnect(c, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
    QObject::disconnect(c, SIGNAL(enabled(Connector*)), this, SIGNAL(connectorEnabled(Connector*)));
    QObject::disconnect(c, SIGNAL(disabled(Connector*)), this, SIGNAL(connectorDisabled(Connector*)));
}

void Box::resizeEvent(QResizeEvent *)
{
    Q_EMIT changed(this);
}


int Box::countInputs()
{
    return input.size();
}

int Box::countOutputs()
{
    return output.size();
}

ConnectorIn* Box::getInput(const unsigned int index)
{
    assert(index < input.size());
    return input[index];
}

ConnectorOut* Box::getOutput(const unsigned int index)
{
    assert(index < output.size());
    return output[index];
}

ConnectorIn* Box::getInput(const std::string& uuid)
{
    BOOST_FOREACH(ConnectorIn* in, input) {
        if(in->UUID() == uuid) {
            return in;
        }
    }

    return NULL;
}

ConnectorOut* Box::getOutput(const std::string& uuid)
{
    BOOST_FOREACH(ConnectorOut* out, output) {
        if(out->UUID() == uuid) {
            return out;
        }
    }

    return NULL;
}

void Box::init(const QPoint& pos)
{
    move(pos);

    //    QBoxLayout* layout = new QVBoxLayout;
    //    ui->content->setLayout(layout);

    makeThread();
    content_->fill(ui->content);
}

BoxedObject* Box::getContent()
{
    return content_;
}

Box::~Box()
{
    stop();

    delete worker_;
    delete content_;
}

bool Box::eventFilter(QObject* o, QEvent* e)
{
    QMouseEvent* em = dynamic_cast<QMouseEvent*>(e);

    if(o == ui->label) {
        if(e->type() == QEvent::MouseButtonDblClick) {
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
    return false;
}

void Box::enabledChange(bool val)
{
    if(val)  {
        content_->enable();
    } else {
        content_->disable();
    }
}

void Box::paintEvent(QPaintEvent*)
{
    bool is_error = content_->isError() && content_->errorLevel() == Displayable::EL_ERROR;
    bool is_warn = content_->isError() && content_->errorLevel() == Displayable::EL_WARNING;

    bool error_change = ui->boxframe->property("error").toBool() != is_error;
    bool warning_change = ui->boxframe->property("warning").toBool() != is_warn;

    ui->boxframe->setProperty("error", is_error);
    ui->boxframe->setProperty("warning", is_warn);

    if(error_change || warning_change) {
        if(is_error) {
            setLabel(QString("ERROR: ") + objectName());
            ui->label->setToolTip(content_->errorMessage().c_str());
        } else if(is_warn) {
            setLabel(QString("WARNING: ") + objectName());
            ui->label->setToolTip(content_->errorMessage().c_str());
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

    Q_EMIT moved(this, delta.x(), delta.y());
}

void Box::registered()
{
    foreach(ConnectorIn* i, input) {
        Q_EMIT connectorCreated(i);
    }
    foreach(ConnectorOut* i, output) {
        Q_EMIT connectorCreated(i);
    }
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
    mimeData->setParent(this);
    mimeData->setData(Box::MIME_MOVE, QByteArray());
    mimeData->setUserData(0, new MoveOffset(offset));
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
    Qt::DropAction action = drag->exec();

    QPoint end_pos = pos();

    Command::Ptr cmd(new command::MoveBox(this, start_pos, end_pos));
    CommandDispatcher::execute(cmd);

    if(action == Qt::IgnoreAction) {
        CommandDispatcher::instance().undo();
    }
}

QPixmap Box::makePixmap(const std::string& label)
{
    int border = 3;

    QImage img(QSize(80, 80), QImage::Format_ARGB32);
    QPainter painter(&img);
    painter.fillRect(QRect(QPoint(0,0), img.size()), Qt::white);
    painter.setPen(QPen(Qt::red, border));
    painter.drawRect(QRect(QPoint(border/2,border/2), img.size() - QSize(border,border)));
    painter.drawText(QPoint(5, 40), label.c_str());
    return QPixmap::fromImage(img);
}

void Box::deleteBox()
{
    Command::Ptr cmd(new command::DeleteBox(this));
    CommandDispatcher::execute(cmd);
}

void Box::refreshStylesheet()
{
    ui->boxframe->style()->unpolish(ui->boxframe);
    ui->boxframe->style()->polish(ui->boxframe);
}

void Box::eventModelChanged()
{
    content_->updateDynamicGui(ui->content);
}

void Box::tick()
{
    if(state->enabled) {
        Q_EMIT tickRequest();
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
        worker_ = new BoxWorker(this);

        QObject::connect(this, SIGNAL(tickRequest()), worker_, SLOT(tick()));
        BOOST_FOREACH(ConnectorIn* in, input) {
            QObject::connect(in, SIGNAL(messageArrived(ConnectorIn*)), worker_, SLOT(forwardMessage(ConnectorIn*)));
        }

        makeThread();
    }
}

void Box::minimizeBox(bool minimize)
{
    if(minimize) {
        ui->frame->hide();
        ui->label->hide();
        ui->killbtn->hide();
        ui->boxframe->setProperty("content_minimized", true);
        ui->minimizebtn->setIcon(maximize_icon_);
        state->minimized = true;
    } else {
        ui->frame->show();
        ui->label->show();
        ui->killbtn->show();
        ui->boxframe->setProperty("content_minimized", false);
        ui->minimizebtn->setIcon(minimize_icon_);
        state->minimized = false;
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

    memento->boxed_state = content_->getState();

    return memento;
}

void Box::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    *state = *m;
    state->parent = this;
    if(m->boxed_state != NULL) {
        content_->setState(m->boxed_state);
    }

    minimizeBox(state->minimized);
    ui->minimizebtn->setChecked(state->minimized);

    enableContent(state->enabled);
    ui->enablebtn->setChecked(state->enabled);

    setLabel(state->label_);
    ui->label->setToolTip(state->uuid_.c_str());
}

Command::Ptr Box::removeAllConnectionsCmd()
{
    command::Meta::Ptr cmd(new command::Meta);

    BOOST_FOREACH(ConnectorIn* i, input) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }
    BOOST_FOREACH(ConnectorOut* i, output) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }

    return cmd;
}

Command::Ptr Box::removeAllOutputsCmd()
{
    command::Meta::Ptr cmd(new command::Meta);

    BOOST_FOREACH(ConnectorOut* i, output) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }

    return cmd;
}

Command::Ptr Box::removeAllInputsCmd()
{
    command::Meta::Ptr cmd(new command::Meta);

    BOOST_FOREACH(ConnectorIn* i, input) {
        if(i->isConnected()) {
            cmd->add(i->removeAllConnectionsCmd());
        }
    }
    return cmd;
}


