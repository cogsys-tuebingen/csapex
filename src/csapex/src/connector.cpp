/// HEADER
#include <csapex/connector.h>

/// COMPONENT
#include <csapex/design_board.h>
#include <csapex/box.h>
#include <csapex/boxed_object.h>
#include <csapex/command_dispatcher.h>
#include <csapex/command_add_connection.h>
#include <csapex/command_move_connection.h>

/// SYSTEM
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <QDragEnterEvent>

using namespace csapex;

const QString Connector::MIME_CREATE = "csapex/connector/create";
const QString Connector::MIME_MOVE = "csapex/connector/move";

Connector::Connector(Box* parent, const std::string& type, int sub_id)
    : parent_widget(parent), designer(NULL), buttons_down_(0)
{
    setBox(parent);

    findParents();
    setFocusPolicy(Qt::NoFocus);
    setAcceptDrops(true);

    std::stringstream ss;
    ss << box_->UUID() << "_" << type << "_" << sub_id;

    uuid_ = ss.str();

    setContextMenuPolicy(Qt::PreventContextMenu);
    setType(ConnectionType::makeDefault());

    setFixedSize(16,16);

    setMouseTracking(true);
}

Connector::~Connector()
{
}

void Connector::errorEvent(bool error, ErrorLevel level)
{
    box_->getContent()->setError(error, "Connector Error", level);
}

std::string Connector::UUID()
{
    return uuid_;
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

void Connector::validateConnections()
{

}

void Connector::removeAllConnectionsUndoable()
{
    if(isConnected()) {
        Command::Ptr cmd = removeAllConnectionsCmd();
        CommandDispatcher::execute(cmd);
    }
}

void Connector::disable()
{
    setEnabled(false);
    Q_EMIT disabled(this);
}

void Connector::enable()
{
    setEnabled(true);
    Q_EMIT enabled(this);
}

void Connector::findParents()
{
    QWidget* tmp = this;
    while(tmp != NULL) {
        if(dynamic_cast<csapex::Box*>(tmp)) {
            box_ = dynamic_cast<csapex::Box*>(tmp);
        } else if(dynamic_cast<csapex::DesignBoard*>(tmp)) {
            designer = dynamic_cast<csapex::DesignBoard*>(tmp);
        }
        tmp = tmp->parentWidget();
    }
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
        if(from->canConnectTo(this)) {
            if(canConnectTo(from)) {
                e->acceptProposedAction();
            }
        }
    } else if(e->mimeData()->text() == Connector::MIME_MOVE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        if(from->targetsCanConnectTo(this)) {
            e->acceptProposedAction();
        }
    }
}

void Connector::dragMoveEvent(QDragMoveEvent* e)
{
    Q_EMIT(connectionStart());
    if(e->mimeData()->text() == Connector::MIME_CREATE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());
        Q_EMIT(connectionInProgress(this, from));

    } else if(e->mimeData()->text() == Connector::MIME_MOVE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        from->connectionMovePreview(this);
    }
}

void Connector::dropEvent(QDropEvent* e)
{
    if(e->mimeData()->text() == Connector::MIME_CREATE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        if(from && from != this) {
            Command::Ptr cmd(new command::AddConnection(this, from));
            CommandDispatcher::execute(cmd);
        }
    } else if(e->mimeData()->text() == Connector::MIME_MOVE) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        if(from && from != this) {
            Command::Ptr cmd(new command::MoveConnection(from, this));
            CommandDispatcher::execute(cmd);
            e->setDropAction(Qt::MoveAction);
        }
    }
}

void Connector::mousePressEvent(QMouseEvent* e)
{
    buttons_down_ = e->buttons();
}

void Connector::mouseMoveEvent(QMouseEvent* e)
{
    if(buttons_down_ == Qt::NoButton) {
        return;
    }

    bool left = (buttons_down_ & Qt::LeftButton) != 0;
    bool right = (buttons_down_ & Qt::RightButton) != 0;

    bool full_input = isInput() && this->isConnected();
    bool create = left && !full_input;
    bool move = (right && isConnected()) || (left && full_input);

    if(create || move) {
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;

        if(move) {

            mimeData->setText(Connector::MIME_MOVE);
            mimeData->setParent(this);
            drag->setMimeData(mimeData);

            disable();

            drag->exec();

            enable();

        } else {
            mimeData->setText(Connector::MIME_CREATE);
            mimeData->setParent(this);
            drag->setMimeData(mimeData);

            drag->exec();
        }

        e->accept();

        Q_EMIT connectionDone();
        buttons_down_ = Qt::NoButton;
    }
    e->accept();
}

void Connector::mouseReleaseEvent(QMouseEvent* e)
{
    buttons_down_ = e->buttons();

    if(e->button() == Qt::MiddleButton) {
        removeAllConnectionsUndoable();
    } else if(e->button() == Qt::RightButton) {
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

    validateConnections();
}

ConnectionType::ConstPtr Connector::getType() const
{
    return type_;
}
