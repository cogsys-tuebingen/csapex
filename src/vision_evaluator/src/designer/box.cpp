/// HEADER
#include "box.h"

/// COMPONENT
#include "ui_box.h"
#include "boxed_object.h"
#include "box_manager.h"
#include "command_move_box.h"
#include "command_delete_box.h"
#include "command_meta.h"

/// SYSTEM
#include <QDragMoveEvent>
#include <QMenu>
#include <iostream>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>

using namespace vision_evaluator;

const QString Box::MIME = "vision_evaluator/box";
const QString Box::MIME_MOVE = "vision_evaluator/box/move";

void Box::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Flow;
    out << YAML::BeginMap;
    out << YAML::Key << "type";
    out << YAML::Value << type_;
    out << YAML::Key << "uuid";
    out << YAML::Value << uuid_;
    out << YAML::Key << "pos";
    out << YAML::Value << YAML::BeginSeq << parent->pos().x() << parent->pos().y() << YAML::EndSeq;

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
    if(node.FindValue("state")) {
        const YAML::Node& state_map = node["state"];
        boxed_state = parent->content_->getState();
        boxed_state->readYaml(state_map);
    }
}


Box::Box(BoxedObject* content, const std::string& uuid, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), state(new State(this)), down_(false)
{
    ui->setupUi(this);

    minimize_icon_ = ui->minimizebtn->icon();
    maximize_icon_ = QIcon(":/maximize.png");

    content_ = content;
    content_->setBox(this);

    ui->enablebtn->setCheckable(content_->canBeDisabled());

    setFocusPolicy(Qt::ClickFocus);

    state->uuid_ = uuid;

    setObjectName(ui->enablebtn->text());

    ui->content->installEventFilter(this);

    connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));
    connect(ui->enablebtn, SIGNAL(toggled(bool)), this, SLOT(enableContent(bool)));

    connect(ui->closebtn, SIGNAL(clicked()), this, SLOT(deleteBox()));
    connect(ui->minimizebtn, SIGNAL(toggled(bool)), this, SLOT(minimizeBox(bool)));

    connect(content, SIGNAL(modelChanged()), this, SLOT(eventModelChanged()), Qt::QueuedConnection);
}

void Box::enableContent(bool enable)
{
    content_->enable(enable);
}


YAML::Emitter& Box::save(YAML::Emitter& out) const
{
    state->writeYaml(out);

    return out;
}

void Box::stop()
{
    content_->stop();
}

void Box::setUUID(const std::string& uuid)
{
    state->uuid_ = uuid;
    ui->enablebtn->setText(state->uuid_.c_str());
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

void Box::addInput(ConnectorIn* in)
{
    in->setParent(NULL);
    ui->input_layout->addWidget(in);
    input.push_back(in);

    QObject::connect(in, SIGNAL(messageArrived(ConnectorIn*)), this, SIGNAL(messageArrived(ConnectorIn*)));
    QObject::connect(in, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(forwardMessage(ConnectorIn*)));
    QObject::connect(in, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::connect(in, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));

    Q_EMIT connectorCreated(in);
}

void Box::addOutput(ConnectorOut* out)
{
    out->setParent(NULL);
    ui->output_layout->addWidget(out);
    output.push_back(out);

    QObject::connect(out, SIGNAL(connectionFormed(ConnectorOut*,ConnectorIn*)), this, SIGNAL(connectionFormed(ConnectorOut*,ConnectorIn*)));
    QObject::connect(out, SIGNAL(connectionDestroyed(ConnectorOut*,ConnectorIn*)), this, SIGNAL(connectionDestroyed(ConnectorOut*,ConnectorIn*)));
    QObject::connect(out, SIGNAL(messageSent(ConnectorOut*)), this, SIGNAL(messageSent(ConnectorOut*)));
    QObject::connect(out, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::connect(out, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));

    Q_EMIT connectorCreated(out);
}

void Box::removeInput(ConnectorIn *in)
{
    std::vector<ConnectorIn*>::iterator it;
    it = std::find(input.begin(), input.end(), in);

    assert(*it == in);

    QObject::disconnect(in, SIGNAL(messageArrived(ConnectorIn*)), this, SIGNAL(messageArrived(ConnectorIn*)));
    QObject::disconnect(in, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::disconnect(in, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
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
    QObject::disconnect(out, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::disconnect(out, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
    out->deleteLater();
    output.erase(it);
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

    content_->fill(ui->content);
}

BoxedObject* Box::getContent()
{
    return content_;
}

Box::~Box()
{
    BoxManager::instance().setDirty(true);
}

bool Box::eventFilter(QObject* o, QEvent* e)
{
    QMouseEvent* em = dynamic_cast<QMouseEvent*>(e);
    if(o == ui->content) {
        if(e->type() == QEvent::MouseButtonPress && em->button() == Qt::LeftButton) {
            down_ = true;
        } else if(e->type() == QEvent::MouseButtonRelease && em->button() == Qt::LeftButton) {
            down_ = false;
        } else if(e->type() == QEvent::MouseMove) {
            if(down_) {
                e->ignore();

                QPoint offset = ui->content->geometry().topLeft();
                startDrag(-em->pos() - offset);

                down_ = false;
                return true;
            }
        }
    }

    return false;
}

void Box::enabledChange(bool val)
{
    if(val)  {
        content_->enable();
        BoxManager::instance().setDirty(true);
    } else {
        content_->disable();
    }
}

void Box::paintEvent(QPaintEvent* e)
{
    ui->enablebtn->setText(objectName());

    bool change = ui->boxframe->property("error").toBool() != content_->isError();
    ui->boxframe->setProperty("error",content_->isError());

    if(change) {
        if(content_->isError()) {
            ui->enablebtn->setText("ERROR: " + objectName());
        } else {
            ui->enablebtn->setText(objectName());
        }

        refreshStylesheet();
    }

    resize(sizeHint());
}

void Box::mousePressEvent(QMouseEvent* e)
{
    if(e->button() == Qt::LeftButton) {
        startDrag(-e->pos());
    }
}

void Box::moveEvent(QMoveEvent* e)
{
    QPoint delta = (e->pos() - e->oldPos());
    if(delta.x() == 0 && delta.y() == 0) {
        return;
    }
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

void Box::focusInEvent(QFocusEvent *e)
{
    ui->boxframe->setProperty("focused",true);
    refreshStylesheet();
}

void Box::focusOutEvent(QFocusEvent *e)
{
    ui->boxframe->setProperty("focused",false);
    refreshStylesheet();
}

void Box::keyPressEvent(QKeyEvent *e)
{
    if(hasFocus()) {
        if(e->key() == Qt::Key_Delete || e->key() == Qt::Key_Backspace) {
            deleteBox();
        }
    }
}

void Box::startDrag(QPoint offset)
{
    QDrag* drag = new QDrag(this);
    QMimeData* mimeData = new QMimeData;
    mimeData->setText(Box::MIME_MOVE);
    mimeData->setParent(this);
    mimeData->setUserData(0, new MoveOffset(offset));
    drag->setMimeData(mimeData);

    QPoint start_pos = pos();
    drag->exec();
    QPoint end_pos = pos();

    Command::Ptr cmd(new command::MoveBox(this, start_pos, end_pos));
    BoxManager::instance().execute(cmd);

    Q_EMIT moved(this);
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
    content_->updateGui(ui->content);
}

void Box::forwardMessage(ConnectorIn *source)
{
//    try {
        content_->messageArrived(source);
        content_->setError(false);
//    } catch(cv::Exception& cve) {
//        content_->setError(true, cve.what());
//    } catch(std::runtime_error& err){
//        content_->setError(true, err.what());
//    }
}

void Box::minimizeBox(bool minimize)
{
    if(minimize) {
        ui->frame->hide();
        ui->enablebtn->setText("");
        ui->boxframe->setProperty("content_minimized", true);
        ui->minimizebtn->setIcon(maximize_icon_);
    } else {
        ui->frame->show();
        ui->enablebtn->setText(objectName());
        ui->boxframe->setProperty("content_minimized", false);
        ui->minimizebtn->setIcon(minimize_icon_);
    }

    refreshStylesheet();

    resize(sizeHint());
}

Memento::Ptr Box::getState() const
{
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


