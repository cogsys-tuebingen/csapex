/// HEADER
#include <csapex/view/widgets/port.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/command/add_connection.h>
#include <csapex/command/command_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/model/connection.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/widgets/message_preview_widget.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/signal/slot.h>
#include <csapex/csapex_mime.h>

/// SYSTEM
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <QDrag>
#include <QWidget>
#include <QDragEnterEvent>
#include <QMimeData>
#include <QHelpEvent>
#include <QEvent>
#include <QInputDialog>
#include <QApplication>
#include <QTimer>

using namespace csapex;

Q_DECLARE_METATYPE(ConnectorPtr)

Port::Port(QWidget* parent) : QFrame(parent), refresh_style_sheet_(false), minimized_(false), flipped_(false), hovered_(false), buttons_down_(0)
{
    setFlipped(flipped_);

    setFocusPolicy(Qt::NoFocus);
    setAcceptDrops(true);

    setContextMenuPolicy(Qt::PreventContextMenu);

    setMinimizedSize(minimized_);

    setEnabled(true);

    double_click_timer_ = new QTimer(this);

    QObject::connect(double_click_timer_, &QTimer::timeout, this, &Port::mouseClickEvent);

    double_click_timer_->setInterval(200);
}

Port::Port(ConnectorPtr adaptee, QWidget* parent) : Port(parent)
{
    adaptee_ = adaptee;

    createToolTip();

    connections_.push_back(adaptee_->enabled_changed.connect([this](bool e) { setEnabledFlag(e); }));

    setProperty("optional", adaptee_->isOptional());

    setProperty("type", QString::fromStdString(port_type::name(adaptee_->getConnectorType())));
}

Port::~Port()
{
    for (auto& c : connections_) {
        c.disconnect();
    }
}

bool Port::event(QEvent* e)
{
    if (e->type() == QEvent::ToolTip) {
        createToolTip();
    }

    return QWidget::event(e);
}

bool Port::isOutput() const
{
    if (!adaptee_) {
        return false;
    }
    return adaptee_->isOutput();
}

bool Port::isInput() const
{
    if (!adaptee_) {
        return false;
    }
    return adaptee_->isInput();
}

ConnectorPtr Port::getAdaptee() const
{
    return adaptee_;
}

void Port::paintEvent(QPaintEvent* e)
{
    if (refresh_style_sheet_) {
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

    if (mini) {
        setFixedSize(8, 8);
    } else {
        setFixedSize(16, 16);
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

bool Port::isHovered() const
{
    return hovered_;
}

void Port::setEnabledFlag(bool processing_enabled)
{
    bool enabled = processing_enabled;
    if (SlotPtr slot = std::dynamic_pointer_cast<Slot>(adaptee_)) {
        if (slot->isActive()) {
            enabled = true;
        }
    }
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
    if (!adaptee_) {
        return;
    }

    std::string tooltip = adaptee_->makeStatusString();

    setToolTip(QString::fromStdString(tooltip));
}

void Port::startDrag()
{
    if (!adaptee_) {
        return;
    }

    Q_EMIT mouseOut(this);

    bool left = (buttons_down_ & Qt::LeftButton) != 0;
    bool right = (buttons_down_ & Qt::RightButton) != 0;

    bool full_input = (adaptee_->isInput() && adaptee_->isConnected());
    bool create = left && !full_input;
    bool move = (right && adaptee_->isConnected()) || (left && full_input);

    if (create || move) {
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;

        if (move) {
            mimeData->setData(QString::fromStdString(csapex::mime::connection_move), QByteArray());
            mimeData->setProperty("Connector", qVariantFromValue(adaptee_));

            drag->setMimeData(mimeData);

            drag->exec();

        } else {
            mimeData->setData(QString::fromStdString(csapex::mime::connection_create), QByteArray());
            mimeData->setProperty("Connector", qVariantFromValue(adaptee_));

            drag->setMimeData(mimeData);

            drag->exec();
        }

        //        adaptee_->connection_added_to(adaptee_);
        buttons_down_ = Qt::NoButton;
    }
}

void Port::mouseMoveEvent(QMouseEvent* e)
{
    if (buttons_down_ == Qt::NoButton) {
        return;
    }

    startDrag();

    e->accept();
}

void Port::mouseDoubleClickEvent(QMouseEvent* e)
{
    if (!adaptee_) {
        return;
    }

    e->accept();

    double_click_timer_->stop();

    bool ok = false;
    QString label = QInputDialog::getText(QApplication::activeWindow(), "Rename Port", "Enter a new name", QLineEdit::Normal, QString::fromStdString(adaptee_->getLabel()), &ok);
    if (ok) {
        changePortRequest(label);
    }
    buttons_down_ = e->buttons();
}

void Port::mouseClickEvent()
{
    startDrag();

    buttons_down_ = Qt::NoButton;
}

void Port::mouseReleaseEvent(QMouseEvent* e)
{
    double_click_timer_->setSingleShot(true);
    double_click_timer_->start();

    if (e->button() == Qt::MiddleButton) {
        Q_EMIT removeConnectionsRequest();
    }

    e->accept();
}

void Port::dragEnterEvent(QDragEnterEvent* e)
{
    if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create))) {
        ConnectorPtr from = e->mimeData()->property("Connector").value<ConnectorPtr>();
        if (from == adaptee_) {
            return;
        }

        bool compatible = Connection::isCompatibleWith(from.get(), adaptee_.get()) && Connection::isCompatibleWith(adaptee_.get(), from.get());

        bool shift = Qt::ShiftModifier & QApplication::keyboardModifiers();
        if (compatible || shift) {
            addConnectionPreview(from);
            e->acceptProposedAction();
        }
    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        ConnectorPtr from = e->mimeData()->property("Connector").value<ConnectorPtr>();

        if (Connection::targetsCanBeMovedTo(from.get(), adaptee_.get())) {
            moveConnectionPreview(from);
            e->acceptProposedAction();
        }
    }
}

void Port::dragMoveEvent(QDragMoveEvent* e)
{
    if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create))) {
        e->acceptProposedAction();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        e->acceptProposedAction();
    }
}

void Port::dropEvent(QDropEvent* e)
{
    if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create))) {
        ConnectorPtr from = e->mimeData()->property("Connector").value<ConnectorPtr>();
        if (from && from != adaptee_) {
            addConnectionRequest(from);
        }
    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        ConnectorPtr from = e->mimeData()->property("Connector").value<ConnectorPtr>();
        if (from) {
            moveConnectionRequest(from);
            e->setDropAction(Qt::MoveAction);
        }
    }
}

void Port::enterEvent(QEvent* /*e*/)
{
    hovered_ = true;
    Q_EMIT mouseOver(this);
}

void Port::leaveEvent(QEvent* /*e*/)
{
    hovered_ = false;
    Q_EMIT mouseOut(this);
}

void Port::mousePressEvent(QMouseEvent* e)
{
    buttons_down_ = e->buttons();
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_port.cpp"
