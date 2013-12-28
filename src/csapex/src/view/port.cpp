/// HEADER
#include <csapex/view/port.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/add_connection.h>
#include <csapex/command/move_connection.h>

/// SYSTEM
#include <sstream>
#include <stdexcept>
#include <QDragEnterEvent>

using namespace csapex;

Port::Port(Connectable *adaptee)
    : adaptee_(adaptee), refresh_style_sheet_(false), minimized_(false), buttons_down_(0)
{
    if(adaptee_) {
        adaptee_->setPort(this);
        setToolTip(adaptee_->UUID().c_str());
    }

    setFixedSize(16, 16);

    setFocusPolicy(Qt::NoFocus);
    setAcceptDrops(true);

    setContextMenuPolicy(Qt::PreventContextMenu);

    setMinimizedSize(minimized_);

    setMouseTracking(true);
}


bool Port::canOutput() const
{
    return adaptee_->canOutput();
}

bool Port::canInput() const
{
    return adaptee_->canInput();
}

bool Port::isOutput() const
{
    return adaptee_->isOutput();
}

bool Port::isInput() const
{
    return adaptee_->isInput();
}

Connectable* Port::getAdaptee() const
{
    return adaptee_;
}



void Port::paintEvent(QPaintEvent *e)
{
    if(refresh_style_sheet_) {
        refresh_style_sheet_ = false;
        setStyleSheet(styleSheet());
    }
    QFrame::paintEvent(e);
}

void Port::refreshStylesheet()
{
    refresh_style_sheet_ = true;
}

void Port::setMinimizedSize(bool mini)
{
    minimized_ = mini;

    if(mini) {
        setFixedSize(8,8);
    } else {
        setFixedSize(16,16);
    }
}

bool Port::isMinimizedSize() const
{
    return minimized_;
}

void Port::mouseMoveEvent(QMouseEvent* e)
{
    std::stringstream tt;
    tt << adaptee_->UUID() << " (" << adaptee_->getCount() << ")";
    setToolTip(tt.str().c_str());

    if(buttons_down_ == Qt::NoButton) {
        return;
    }

    bool left = (buttons_down_ & Qt::LeftButton) != 0;
    bool right = (buttons_down_ & Qt::RightButton) != 0;

    bool create = adaptee_->shouldCreate(left, right);
    bool move = adaptee_->shouldMove(left, right);

    if(create || move) {
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;

        if(move) {
            mimeData->setData(Connectable::MIME_MOVE_CONNECTIONS, QByteArray());
            mimeData->setParent(adaptee_);
            drag->setMimeData(mimeData);

            adaptee_->disable();

            drag->exec();

            adaptee_->enable();

        } else {
            mimeData->setData(Connectable::MIME_CREATE_CONNECTION, QByteArray());
            mimeData->setParent(adaptee_);
            drag->setMimeData(mimeData);

            drag->exec();
        }

        e->accept();

        Q_EMIT adaptee_->connectionDone();
        buttons_down_ = Qt::NoButton;
    }
    e->accept();
}

void Port::mouseReleaseEvent(QMouseEvent* e)
{
    buttons_down_ = e->buttons();

    if(e->button() == Qt::MiddleButton) {
        adaptee_->removeAllConnectionsUndoable();
    } else if(e->button() == Qt::RightButton) {
        adaptee_->removeAllConnectionsUndoable();
    }

    e->accept();
}

void Port::dragEnterEvent(QDragEnterEvent* e)
{
    if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        Connectable* from = dynamic_cast<Connectable*>(e->mimeData()->parent());
        if(from == adaptee_) {
            return;
        }

        if(from->canConnectTo(adaptee_, false)) {
            if(adaptee_->canConnectTo(from, false)) {
                e->acceptProposedAction();
            }
        }
    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* original = dynamic_cast<Connectable*>(e->mimeData()->parent());

        if(original->targetsCanBeMovedTo(adaptee_)) {
            e->acceptProposedAction();
        }
    }
}

void Port::dragMoveEvent(QDragMoveEvent* e)
{
    Q_EMIT(adaptee_->connectionStart());
    if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        Connectable* from = dynamic_cast<Connectable*>(e->mimeData()->parent());
        Q_EMIT(adaptee_->connectionInProgress(adaptee_, from));

    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* from = dynamic_cast<Connectable*>(e->mimeData()->parent());

        from->connectionMovePreview(adaptee_);
    }
}

void Port::dropEvent(QDropEvent* e)
{
    if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        Connectable* from = dynamic_cast<Connectable*>(e->mimeData()->parent());

        if(from && from != adaptee_) {
            adaptee_->getNode()->getCommandDispatcher()->execute(Command::Ptr(new command::AddConnection(adaptee_->UUID(), from->UUID())));
        }
    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* from = dynamic_cast<Connectable*>(e->mimeData()->parent());

        if(from) {
            Command::Ptr cmd(new command::MoveConnection(from, adaptee_));
            adaptee_->getNode()->getCommandDispatcher()->execute(cmd);
            e->setDropAction(Qt::MoveAction);
        }
    }
}

void Port::mousePressEvent(QMouseEvent* e)
{
    buttons_down_ = e->buttons();
}


QPoint Port::topLeft()
{
    QWidget* parent = parentWidget();
    while(parent->objectName() != "DesignBoard") {
        parent = parent->parentWidget();
    };

    return mapTo(parent, parent->pos());
}

QPoint Port::centerPoint()
{
    QPoint offset = 0.5 * (geometry().bottomRight() - geometry().topLeft());
    return topLeft() + offset;
}
