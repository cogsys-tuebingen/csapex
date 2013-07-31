/// HEADER
#include <csapex/box.h>

/// COMPONENT
#include "ui_box.h"
#include <csapex/boxed_object.h>
#include <csapex/box_manager.h>
#include <csapex/command_move_box.h>
#include <csapex/command_delete_box.h>
#include <csapex/command_meta.h>

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
    : QWidget(parent), ui(new Ui::Box), state(new State(this)), private_thread_(NULL), worker_(new BoxWorker(this)), down_(false)
{
    ui->setupUi(this);

    minimize_icon_ = ui->minimizebtn->icon();
    maximize_icon_ = QIcon(":/maximize.png");

    content_ = content;
    content_->setBox(this);

    ui->enablebtn->setCheckable(content_->canBeDisabled());

    setFocusPolicy(Qt::ClickFocus);

    state->uuid_ = uuid;

    setObjectName(uuid.c_str());
    setLabel(uuid);

    ui->content->installEventFilter(this);
    ui->label->installEventFilter(this);

    ui->enablebtn->setIcon(content->getIcon());

    timer_ = new QTimer();
    timer_->setInterval(100);
    timer_->start();

    state->minimized = false;

    QObject::connect(timer_, SIGNAL(timeout()), worker_, SLOT(tick()));

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

void Box::enableContent(bool enable)
{
    if(enable && !timer_->isActive()) {
        timer_->start();
    } else if(!enable && timer_->isActive()) {
        timer_->stop();
    }

    state->enabled = enable;

    content_->enable(enable);
}

YAML::Emitter& Box::save(YAML::Emitter& out) const
{
    state->writeYaml(out);

    return out;
}

void Box::stop()
{
    if(private_thread_) {
        private_thread_->quit();
        private_thread_->wait(1000);
        if(private_thread_->isRunning()) {
            std::cout << "terminate thread" << std::endl;
            private_thread_->terminate();
        }
    }
}

void BoxWorker::forwardMessage(ConnectorIn *source)
{
    if(parent_->isVisible() && parent_->content_->isEnabled()) {
        parent_->content_->messageArrived(source);
        parent_->content_->setError(false);
    }
}

void BoxWorker::tick()
{
    if(parent_->isVisible() && parent_->content_->isEnabled()) {
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
    state->label_ = label.toUtf8().constData();
    ui->label->setText(label);
}

std::string Box::getLabel() const
{
    return ui->label->text().toUtf8().constData();
}

void Box::addInput(ConnectorIn* in)
{
    in->setParent(NULL);
    ui->input_layout->addWidget(in);
    input.push_back(in);

    QObject::connect(in, SIGNAL(messageArrived(ConnectorIn*)), this, SIGNAL(messageArrived(ConnectorIn*)));
    QObject::connect(in, SIGNAL(messageArrived(ConnectorIn*)), worker_, SLOT(forwardMessage(ConnectorIn*)));

    connectConnector(in);

    Q_EMIT connectorCreated(in);
    Q_EMIT changed(this);
}

void Box::addOutput(ConnectorOut* out)
{
    out->setParent(NULL);
    ui->output_layout->addWidget(out);
    output.push_back(out);

    QObject::connect(out, SIGNAL(messageSent(ConnectorOut*)), this, SIGNAL(messageSent(ConnectorOut*)));
    QObject::connect(out, SIGNAL(connectionFormed(ConnectorOut*,ConnectorIn*)), this, SIGNAL(connectionFormed(ConnectorOut*,ConnectorIn*)));
    QObject::connect(out, SIGNAL(connectionDestroyed(ConnectorOut*,ConnectorIn*)), this, SIGNAL(connectionDestroyed(ConnectorOut*,ConnectorIn*)));

    connectConnector(out);

    Q_EMIT connectorCreated(out);
    Q_EMIT changed(this);
}
void Box::removeInput(ConnectorIn *in)
{
    std::vector<ConnectorIn*>::iterator it;
    it = std::find(input.begin(), input.end(), in);

    assert(*it == in);

    QObject::disconnect(in, SIGNAL(messageArrived(ConnectorIn*)), this, SIGNAL(messageArrived(ConnectorIn*)));

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
    QObject::disconnect(out, SIGNAL(messageSent(ConnectorOut*)), this, SIGNAL(messageSent(ConnectorOut*)));

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

void Box::resizeEvent(QResizeEvent *e)
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
                        BoxManager::instance().startPlacingBox(getType(), -start_drag_);
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

void Box::paintEvent(QPaintEvent* e)
{
    bool change = ui->boxframe->property("error").toBool() != content_->isError();
    ui->boxframe->setProperty("error",content_->isError());

    if(change) {
        if(content_->isError()) {
            setLabel(QString("ERROR: ") + objectName());
        } else {
            setLabel(objectName());
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

void Box::keyPressEvent(QKeyEvent *e)
{

}

void Box::startDrag(QPoint offset)
{
    QDrag* drag = new QDrag(this);
    QMimeData* mimeData = new QMimeData;
    mimeData->setText(Box::MIME_MOVE);
    mimeData->setParent(this);
    mimeData->setUserData(0, new MoveOffset(offset));
    drag->setMimeData(mimeData);

    if(Qt::ShiftModifier == QApplication::keyboardModifiers()) {
        BoxManager::instance().startPlacingBox(getType(), offset);
        return;
    }

    QPoint start_pos = pos();
    drag->exec();
    QPoint end_pos = pos();

    Command::Ptr cmd(new command::MoveBox(this, start_pos, end_pos));
    BoxManager::instance().execute(cmd);
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
    BoxManager::instance().execute(cmd);
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

void Box::killContent()
{
    if(private_thread_ && private_thread_->isRunning()) {
        worker_mutex_.lock();

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

        QObject::connect(timer_, SIGNAL(timeout()), worker_, SLOT(tick()));
        BOOST_FOREACH(ConnectorIn* in, input) {
            QObject::connect(in, SIGNAL(messageArrived(ConnectorIn*)), worker_, SLOT(forwardMessage(ConnectorIn*)));
        }

        makeThread();

        worker_mutex_.unlock();
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


