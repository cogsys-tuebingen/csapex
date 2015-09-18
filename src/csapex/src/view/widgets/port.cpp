/// HEADER
#include <csapex/view/widgets/port.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/add_connection.h>
#include <csapex/command/move_connection.h>
#include <csapex/command/command_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/manager/message_renderer_manager.h>
#include <csapex/view/designer/widget_controller.h>
#include <csapex/view/designer/designer_view.h>

/// SYSTEM
#include <sstream>
#include <stdexcept>
#include <QDrag>
#include <QWidget>
#include <QDragEnterEvent>
#include <QMimeData>
#include <QHelpEvent>
#include <QEvent>

using namespace csapex;

Port::Port(CommandDispatcher *dispatcher, WidgetController* widget_controller, ConnectableWeakPtr adaptee, QWidget *parent)
    : QFrame(parent), dispatcher_(dispatcher), widget_controller_(widget_controller), adaptee_(adaptee), refresh_style_sheet_(false), minimized_(false), flipped_(false), buttons_down_(0)
{
    ConnectablePtr adaptee_ptr = adaptee_.lock();
    if(adaptee_ptr) {
        createToolTip();

        connections_.push_back(adaptee_ptr->enabled_changed.connect([this](bool e) { setEnabledFlag(e); }));
        connections_.push_back(adaptee_ptr->connectableError.connect([this](bool error,std::string msg,int level) { setError(error, msg, level); }));

        if(adaptee_ptr->isDynamic()) {
            setProperty("dynamic", true);
        }

    } else {
        std::cerr << "creating empty port!" << std::endl;
    }

    setFlipped(flipped_);

    setFocusPolicy(Qt::NoFocus);
    setAcceptDrops(true);

    setContextMenuPolicy(Qt::PreventContextMenu);

    setMinimizedSize(minimized_);

    setEnabled(true);
}

Port::~Port()
{
    tooltip_connection.disconnect();
    for(auto c : connections_) {
        c.disconnect();
    }
}

bool Port::event(QEvent *e)
{
    if (e->type() == QEvent::ToolTip) {
        ConnectablePtr adaptee = adaptee_.lock();
        if(!adaptee) {
            return false;
        }
        createToolTip();

        if(adaptee->isOutput()) {
            Output* output = dynamic_cast<Output*>(adaptee.get());
            if(output) {
                if(!output->isConnected()) {
                    output->forceSendMessage(true);
                }

                QHelpEvent *helpEvent = static_cast<QHelpEvent *>(e);
                QPoint global_pos = helpEvent->globalPos();
                DesignerView* dview = widget_controller_->getDesignerView();
                QPointF pos = dview->mapToScene(dview->mapFromGlobal(global_pos));
                pos.setY(pos.y() + 50);

                QGraphicsView* view = widget_controller_->getTooltipView("Output");
                view->move(pos.toPoint());
                view->show();

                updateTooltip();

                tooltip_connection = output->messageSent.connect([this](Connectable*) { updateTooltip(); });
            }
        }
    }

    return QWidget::event(e);
}

void Port::updateTooltip()
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return;
    }
    StaticOutput* output = dynamic_cast<StaticOutput*>(adaptee.get());

    if(output) {
        QGraphicsView* view = widget_controller_->getTooltipView("Output");

        if(view->isHidden()) {
            return;
        }

        ConnectionType::ConstPtr msg = output->getMessage();

        if(msg) {

            try {
                MessageRenderer::Ptr renderer = MessageRendererManager::instance().createMessageRenderer(msg);
                if(renderer) {
                    QImage img = renderer->render(msg);

                    auto pm = QPixmap::fromImage(img);
                    view->scene()->clear();
                    view->scene()->addPixmap(pm);
                    view->setMaximumSize(256, 256);
                    view->fitInView(view->scene()->sceneRect(), Qt::KeepAspectRatio);
                }
            } catch(const std::exception& e) {
                view->hide();
            }

        }
    }
}

bool Port::canOutput() const
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return false;
    }
    return adaptee->canOutput();
}

bool Port::canInput() const
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return false;
    }
    return adaptee->canInput();
}

bool Port::isOutput() const
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return false;
    }
    return adaptee->isOutput();
}

bool Port::isInput() const
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return false;
    }
    return adaptee->isInput();
}

ConnectableWeakPtr Port::getAdaptee() const
{
    return adaptee_;
}


void Port::paintEvent(QPaintEvent *e)
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return;
    }
    bool opt = dynamic_cast<Input*>(adaptee.get()) && dynamic_cast<Input*>(adaptee.get())->isOptional();
    setProperty("unconnected", isInput() && !opt && !adaptee->isConnected());
    setProperty("optional", opt);

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
    setError(error, msg, static_cast<int>(ErrorState::ErrorLevel::ERROR));
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

void Port::setEnabledFlag(bool enabled)
{
    setPortProperty("enabled", enabled);
    setPortProperty("disabled", !enabled);
    setEnabled(true);
    refreshStylesheet();
}

void Port::setPortProperty(const std::string& name, bool b)
{
    setProperty(name.c_str(), b);
}

void Port::createToolTip()
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return;
    }

    std::stringstream tooltip;
    tooltip << "UUID: " << adaptee->getUUID().c_str();
    tooltip << ", Type: " << adaptee->getType()->descriptiveName();
    tooltip << ", Messages: " << adaptee->getCount();
    tooltip << ", Enabled: " << adaptee->isEnabled();
    tooltip << ", #: " << adaptee->sequenceNumber();

    Output* o = dynamic_cast<Output*>(adaptee.get());
    if(o) {
        tooltip << ", state: ";
        switch(o->getState()) {
        case Output::State::ACTIVE:
            tooltip << "ACTIVE";
            break;
        case Output::State::IDLE:
            tooltip << "IDLE";
            break;
        case Output::State::RECEIVING:
            tooltip << "RECEIVING";
            break;
        default:
            tooltip << "UNKNOWN";
        }
    }
    setToolTip(tooltip.str().c_str());
}

void Port::startDrag()
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return;
    }

    bool left = (buttons_down_ & Qt::LeftButton) != 0;
    bool right = (buttons_down_ & Qt::RightButton) != 0;

    bool create = adaptee->shouldCreate(left, right);
    bool move = adaptee->shouldMove(left, right);

    adaptee->connectionStart(adaptee.get());

    if(create || move) {
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;

        if(move) {
            mimeData->setData(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS), QByteArray());
            mimeData->setProperty("connectable", qVariantFromValue(static_cast<void*> (adaptee.get())));

            drag->setMimeData(mimeData);

            drag->exec();

        } else {
            mimeData->setData(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION), QByteArray());
            mimeData->setProperty("connectable", qVariantFromValue(static_cast<void*> (adaptee.get())));

            drag->setMimeData(mimeData);

            drag->exec();
        }

        adaptee->connection_added_to(adaptee.get());
        buttons_down_ = Qt::NoButton;
    }
}

void Port::mouseMoveEvent(QMouseEvent* e)
{
    if(buttons_down_ == Qt::NoButton) {
        return;
    }

    startDrag();

    e->accept();
}

void Port::mouseReleaseEvent(QMouseEvent* e)
{
    startDrag();

    buttons_down_ = e->buttons();

    if(e->button() == Qt::MiddleButton) {
        ConnectablePtr adaptee = adaptee_.lock();
        if(!adaptee) {
            return;
        }
        dispatcher_->execute(CommandFactory::removeAllConnectionsCmd(adaptee));
    }

    e->accept();
}

void Port::dragEnterEvent(QDragEnterEvent* e)
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return;
    }
    if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION))) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        if(from == adaptee.get()) {
            return;
        }

        if(from->canConnectTo(adaptee.get(), false)) {
            if(adaptee->canConnectTo(from, false)) {
                e->acceptProposedAction();
                Q_EMIT(adaptee->connectionInProgress(adaptee.get(), from));
            }
        }
    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
        Connectable* original = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());

        if(original->targetsCanBeMovedTo(adaptee.get())) {
            e->acceptProposedAction();
        }
    }
}

void Port::dragMoveEvent(QDragMoveEvent* e)
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return;
    }
    if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION))) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());

        from->connectionMovePreview(adaptee.get());

        e->acceptProposedAction();
    }
}

void Port::dropEvent(QDropEvent* e)
{
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return;
    }
    if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION))) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());

        if(from && from != adaptee.get()) {
            dispatcher_->execute(Command::Ptr(new command::AddConnection(adaptee->getUUID(), from->getUUID())));
        }
    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());

        if(from) {
            Command::Ptr cmd(new command::MoveConnection(from, adaptee.get()));
            dispatcher_->execute(cmd);
            e->setDropAction(Qt::MoveAction);
        }
    }
}

void Port::enterEvent(QEvent */*e*/)
{

}

void Port::leaveEvent(QEvent */*e*/)
{    
    ConnectablePtr adaptee = adaptee_.lock();
    if(!adaptee) {
        return;
    }
    //    QObject::disconnect(this, SLOT(updateTooltip()));

    if(adaptee->isOutput()) {
        Output* output = dynamic_cast<Output*>(adaptee.get());
        if(output) {
            output->forceSendMessage(false);

            tooltip_connection.disconnect();

            widget_controller_->hideTooltipView();
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
    return mapTo(parent,  QPoint(0,0));
}

QPoint Port::centerPoint()
{
    QPoint offset = 0.5 * (geometry().bottomRight() - geometry().topLeft());
    return topLeft() + offset;
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_port.cpp"
