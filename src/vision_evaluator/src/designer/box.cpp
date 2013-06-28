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

    out << YAML::Key << "connections";
    out << YAML::Value << YAML::BeginMap;

    if(!parent->output.empty()) {
        out << YAML::Key << "out";
        out << YAML::Value << YAML::BeginSeq;
        BOOST_FOREACH(ConnectorOut* o, parent->output) {
            out << YAML::BeginMap;
            out << YAML::Key << "uuid";
            out << YAML::Value << o->UUID();
            out << YAML::Key << "targets";
            out << YAML::Value << YAML::BeginSeq;
            for(ConnectorOut::TargetIterator it = o->beginTargets(); it != o->endTargets(); ++it) {
                ConnectorIn* i = *it;
                out << i->UUID();
            }
            out << YAML::EndSeq;

            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
    }

    out << YAML::EndMap;

    out << YAML::EndMap;
}

void Box::State::readYaml(const YAML::Node &node)
{
    if(node.FindValue("state")) {
        const YAML::Node& state_map = node["state"];
        boxed_state = parent->content_->getState();
        boxed_state->readYaml(state_map);
    }

    if(node.FindValue("connections")) {
        const YAML::Node& connections = node["connections"];
        if(connections.FindValue("out")) {
            const YAML::Node& out_list = connections["out"];
            assert(out_list.Type() == YAML::NodeType::Sequence);

            for(unsigned i=0; i<out_list.size(); ++i) {
                const YAML::Node& connector = out_list[i];
                assert(connector.Type() == YAML::NodeType::Map);
                std::string from_uuid;
                connector["uuid"] >> from_uuid;

                const YAML::Node& targets = connector["targets"];
                assert(targets.Type() == YAML::NodeType::Sequence);

                for(unsigned j=0; j<targets.size(); ++j) {
                    std::string to_uuid;
                    targets[j] >> to_uuid;

                    ConnectorOut* from = parent->getOutput(from_uuid);
                    if(from == NULL) {
                        std::cerr << "cannot load connection, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                        continue;
                    }

                    Box* target_box = BoxManager::instance().findConnectorOwner(to_uuid);
                    if(target_box == NULL) {
                        std::cerr << "cannot load connection, connector with uuid '" << to_uuid << "' doesn't exist." << std::endl;
                        continue;
                    }

                    ConnectorIn* to = target_box->getInput(to_uuid);
                    assert(to); // if parent box has been found, this should never happen

                    from->connectForcedWithoutCommand(to);
                }
            }

        }
    }
}


Box::Box(BoxedObject* content, const std::string& uuid, QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), state(new State(this)), down_(false)
{
    ui->setupUi(this);

    content_ = content;
    content_->setBox(this);

    ui->content->setCheckable(content_->canBeDisabled());

    state->uuid_ = uuid;

    setObjectName(ui->content->title());

    ui->content->installEventFilter(this);


    setContextMenuPolicy(Qt::CustomContextMenu);

    connect(ui->content, SIGNAL(toggled(bool)), this, SIGNAL(toggled(bool)));
    connect(ui->content, SIGNAL(toggled(bool)), this, SLOT(enableContent(bool)));

    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
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
    ui->content->setTitle(state->uuid_.c_str());
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
    QObject::connect(in, SIGNAL(connectionInProgress(Connector*,Connector*)), this, SIGNAL(connectionInProgress(Connector*,Connector*)));
    QObject::connect(in, SIGNAL(connectionDone()), this, SIGNAL(connectionDone()));
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

    QBoxLayout* layout = new QVBoxLayout;
    ui->content->setLayout(layout);

    content_->fill(layout);
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
    ui->content->setTitle(objectName());

    bool change = ui->boxframe->property("error").toBool() != content_->isError();
    ui->boxframe->setProperty("error",content_->isError());

    if(change) {
        if(content_->isError()) {
            ui->content->setTitle("ERROR: " + objectName());
        } else {
            ui->content->setTitle(objectName());
        }

        ui->boxframe->style()->unpolish(ui->boxframe);
        ui->boxframe->style()->polish(ui->boxframe);
        ui->boxframe->update();
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

void Box::showContextMenu(const QPoint& pos)
{
    QPoint globalPos = mapToGlobal(pos);

    QString remove_txt("delete");

    QMenu menu;
    menu.addAction(remove_txt);

    QAction* selectedItem = menu.exec(globalPos);

    if(selectedItem) {
        if(selectedItem->text() == remove_txt) {
            Command::Ptr cmd(new command::DeleteBox(this));
            BoxManager::instance().execute(cmd);
        }
    }
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
    content_->setState(m->boxed_state);
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
