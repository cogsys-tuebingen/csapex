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
    : dispatcher_(dispatcher), adaptee_(adaptee), refresh_style_sheet_(false), minimized_(false), flipped_(false), buttons_down_(0), guard_(0xDEADBEEF)
{
    if(adaptee_) {
        adaptee_->setCommandDispatcher(dispatcher);

        createToolTip();

        QObject::connect(adaptee, SIGNAL(blocked(bool)), this, SLOT(setBlocked(bool)));
        QObject::connect(adaptee, SIGNAL(destroyed()), this, SLOT(deleteLater()));
        QObject::connect(adaptee, SIGNAL(connectableError(bool,std::string,int)), this, SLOT(setError(bool, std::string, int)));
        QObject::connect(adaptee, SIGNAL(enabled(bool)), this, SLOT(setEnabledFlag(bool)));

    } else {
        std::cerr << "creating empty port!" << std::endl;
    }

    setFixedSize(16, 16);

    setFocusPolicy(Qt::NoFocus);
    setAcceptDrops(true);

    setContextMenuPolicy(Qt::PreventContextMenu);

    setMinimizedSize(minimized_);

    setEnabled(true);
    setMouseTracking(true);
}

Port::~Port()
{
    guard_ = 0x1;
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

void Port::setError(bool error, const std::string& msg)
{
    setError(error, msg, ErrorState::EL_ERROR);
}

void Port::setError(bool error, const std::string& /*msg*/, int /*level*/)
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

void Port::setBlocked(bool blocked)
{
    setPortProperty("blocked", blocked);
}

void Port::setEnabledFlag(bool enabled)
{
    setPortProperty("enabled", enabled);
    setPortProperty("disabled", !enabled);
    setEnabled(true);
}

void Port::setPortProperty(const std::string& name, bool b)
{
    setProperty(name.c_str(), b);
    refreshStylesheet();
}

void Port::createToolTip()
{
    std::stringstream tooltip;
    tooltip << "UUID: " << adaptee_->getUUID().c_str() << ", Type: " << adaptee_->getType()->name() << ", Messages: " << adaptee_->getCount();
    tooltip << ", Blocked: " << adaptee_->isBlocked() << ", #: " << adaptee_->sequenceNumber();
    setToolTip(tooltip.str().c_str());
}

void Port::mouseMoveEvent(QMouseEvent* e)
{
    createToolTip();

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
            mimeData->setProperty("connectable", qVariantFromValue(static_cast<void*> (adaptee_)));
            drag->setMimeData(mimeData);

            adaptee_->disable();

            drag->exec();

            adaptee_->enable();

        } else {
            mimeData->setData(Connectable::MIME_CREATE_CONNECTION, QByteArray());
            mimeData->setProperty("connectable", qVariantFromValue(static_cast<void*> (adaptee_)));

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
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
         if(from == adaptee_) {
            return;
        }

        if(from->canConnectTo(adaptee_, false)) {
            if(adaptee_->canConnectTo(from, false)) {
                e->acceptProposedAction();
            }
        }
    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* original = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());

        if(original->targetsCanBeMovedTo(adaptee_)) {
            e->acceptProposedAction();
        }
    }
}

void Port::dragMoveEvent(QDragMoveEvent* e)
{
    Q_EMIT(adaptee_->connectionStart());
    if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        Q_EMIT(adaptee_->connectionInProgress(adaptee_, from));

    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());

        from->connectionMovePreview(adaptee_);
    }
}

void Port::dropEvent(QDropEvent* e)
{
    if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());

        if(from && from != adaptee_) {
            dispatcher_->execute(Command::Ptr(new command::AddConnection(adaptee_->getUUID(), from->getUUID())));
        }
    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());

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
