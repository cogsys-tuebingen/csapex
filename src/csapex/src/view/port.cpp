/// HEADER
#include <csapex/view/port.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/add_connection.h>
#include <csapex/command/move_connection.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <sstream>
#include <stdexcept>
#include <QDragEnterEvent>


using namespace csapex;

Port::Port(CommandDispatcher *dispatcher, Connectable *adaptee)
    : dispatcher_(dispatcher), adaptee_(adaptee), refresh_style_sheet_(false), minimized_(false), flipped_(false), buttons_down_(0)
{
    if(adaptee_) {
        adaptee_->setCommandDispatcher(dispatcher);
        adaptee_->setPort(this);
        setToolTip(adaptee_->getUUID().c_str());
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
    setProperty("async", adaptee_->isAsync());

//    if(adaptee_->isInput()) {
//        ConnectorIn* i = dynamic_cast<ConnectorIn*>(adaptee_);
//        setProperty("legacy", i->isLegacy());
//        refreshStylesheet();
//    }

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

void Port::errorEvent(bool /*error*/, const std::string& /*msg*/, ErrorLevel /*level*/)
{
    // TODO: relay to parent (box if applicable)
    //setError(error, msg, level);
}

void Port::errorChanged(bool error)
{
    setProperty("error", error);
    refreshStylesheet();
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

void Port::setFlipped(bool flipped)
{
    flipped_ = flipped;
}

bool Port::isFlipped() const
{
    return flipped_;
}

void Port::setPortProperty(const std::string& name, bool b)
{
    setProperty(name.c_str(), b);
    refreshStylesheet();
}

void Port::mouseMoveEvent(QMouseEvent* e)
{
    std::stringstream tt;
    tt << adaptee_->getUUID() << " (" << adaptee_->getCount() << ")";
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
    std::cout << "port enter: " << e->format() << std::endl;
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
            dispatcher_->execute(Command::Ptr(new command::AddConnection(adaptee_->getUUID(), from->getUUID())));
        }
    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* from = dynamic_cast<Connectable*>(e->mimeData()->parent());

        if(from) {
            Command::Ptr cmd(new command::MoveConnection(from, adaptee_));
            dispatcher_->execute(cmd);
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

    return mapTo(parent,  QPoint(0,0));
}

QPoint Port::centerPoint()
{
    QPoint offset = 0.5 * (geometry().bottomRight() - geometry().topLeft());
    return topLeft() + offset;
}
