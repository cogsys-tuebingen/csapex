/// HEADER
#include "connector.h"

/// COMPONENT
#include "design_board.h"
#include "box.h"
#include "box_manager.h"
#include "command_add_connection.h"

/// SYSTEM
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <QDragEnterEvent>

using namespace vision_evaluator;

const QString Connector::MIME_CREATE = "vision_evaluator/connector/create";
const QString Connector::MIME_MOVE = "vision_evaluator/connector/move";

Connector::Connector(Box* parent, const std::string& type, int sub_id)
    : parent_widget(parent), box_(parent), designer(NULL)
{
    findParents();
    setFocusPolicy(Qt::NoFocus);
    setAcceptDrops(true);

    std::stringstream ss;
    ss << box_->UUID() << "_" << type << "_" << sub_id;

    uuid_ = ss.str();

    setContextMenuPolicy(Qt::PreventContextMenu);
    setType(ConnectionType::makeDefault());
}

Connector::~Connector()
{
}

std::string Connector::UUID()
{
    return uuid_;
}

void Connector::setUUID(const std::string& uuid)
{
    uuid_ = uuid;
}

bool Connector::tryConnect(QObject* other_side)
{
    Connector* c = dynamic_cast<Connector*>(other_side);
    if(c) {
        return tryConnect(c);
    }
    return false;
}

void Connector::removeConnection(QObject* other_side)
{
    Connector* c = dynamic_cast<Connector*>(other_side);
    if(c) {
        removeConnection(c);
    }
}

void Connector::removeAllConnectionsUndoable()
{
    if(isConnected()) {
        Command::Ptr cmd = removeAllConnectionsCmd();
        BoxManager::instance().execute(cmd);
    }
}

void Connector::findParents()
{
    QWidget* tmp = this;
    while(tmp != NULL) {
        if(dynamic_cast<vision_evaluator::Box*>(tmp)) {
            box_ = dynamic_cast<vision_evaluator::Box*>(tmp);
        } else if(dynamic_cast<vision_evaluator::DesignBoard*>(tmp)) {
            designer = dynamic_cast<vision_evaluator::DesignBoard*>(tmp);
        }
        tmp = tmp->parentWidget();
    }
}

bool Connector::hitButton(const QPoint&) const
{
    return false;
}

bool Connector::canConnectTo(Connector* other_side)
{
    bool in_out = (isOutput() && other_side->isInput()) || (isInput() && other_side->isOutput());
    bool compability = getType()->canConnectTo(other_side->getType());
    return in_out && compability;
}

void Connector::dragEnterEvent(QDragEnterEvent* e)
{
    if(e->mimeData()->text() == Connector::MIME_CREATE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());
        if(from->canConnectTo(this) && canConnectTo(from)) {
            e->acceptProposedAction();
        }
    } else if(e->mimeData()->text() == Connector::MIME_MOVE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());
        if(from->canConnectTo(this) && canConnectTo(from)) {
            e->acceptProposedAction();
        }
    }
}

void Connector::dragMoveEvent(QDragMoveEvent* e)
{
    if(e->mimeData()->text() == Connector::MIME_CREATE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        Q_EMIT(connectionInProgress(this, from));
    } else if(e->mimeData()->text() == Connector::MIME_MOVE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        Q_EMIT(connectionInProgress(this, from));
    }
}

void Connector::dropEvent(QDropEvent* e)
{
    if(e->mimeData()->text() == Connector::MIME_CREATE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        if(from && from != this) {
            Command::Ptr cmd(new command::AddConnection(this, from));
            BoxManager::instance().execute(cmd);
        }
    } else if(e->mimeData()->text() == Connector::MIME_MOVE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        if(from && from != this) {
            Command::Ptr cmd(new command::AddConnection(this, from));
            //            BoxManager::instance().execute(cmd);
        }
    }
}

void Connector::mousePressEvent(QMouseEvent* e)
{
    bool create = e->button() == Qt::LeftButton;
    bool move = e->button() == Qt::RightButton && isConnected();

    if(create || move) {
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;

        if(move) {
            ConnectorOut* source;
            if(isOutput()) {
                source = dynamic_cast<ConnectorOut*>(this);
            } else {
                ConnectorIn* self = dynamic_cast<ConnectorIn*>(this);
                source = dynamic_cast<ConnectorOut*>(self->getConnected());
            }

            mimeData->setText(Connector::MIME_MOVE);
            mimeData->setParent(source);
            drag->setMimeData(mimeData);

            Command::Ptr remove = source->removeAllConnectionsCmd();
            BoxManager::instance().execute(remove);

            drag->exec();

            BoxManager::instance().undo();

        } else {
            mimeData->setText(Connector::MIME_CREATE);
            mimeData->setParent(this);
            drag->setMimeData(mimeData);

            drag->exec();
        }

        e->accept();

        Q_EMIT connectionDone();
    }
    e->accept();
}

void Connector::mouseReleaseEvent(QMouseEvent* e)
{
    if(e->button() == Qt::MiddleButton) {
        removeAllConnectionsUndoable();
    }

    e->accept();
}

QPoint Connector::topLeft()
{
    if(box_ == NULL) {
        findParents();
    }

    return box_->geometry().topLeft() + pos();
}

QPoint Connector::centerPoint()
{
    return topLeft() + 0.5 * (geometry().bottomRight() - geometry().topLeft());
}

void Connector::paintEvent(QPaintEvent* e)
{
    setAutoExclusive(false);
    setChecked(isConnected());

    QRadioButton::paintEvent(e);
}

vision_evaluator::Box* Connector::box()
{
    return box_;
}

std::string Connector::getLabel() const
{
    return label_;
}

void Connector::setLabel(const std::string &label)
{
    label_ = label;
}

void Connector::setType(ConnectionType::Ptr type)
{
    type_ = type;
}

ConnectionType::ConstPtr Connector::getType() const
{
    return type_;
}
