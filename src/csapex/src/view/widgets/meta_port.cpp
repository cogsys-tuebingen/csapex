/// HEADER
#include <csapex/view/widgets/meta_port.h>

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/msg/any_message.h>

/// SYSTEM
#include <QDragEnterEvent>
#include <QMimeData>
#include <iostream>
#include <QMenu>
#include <QInputDialog>
#include <QApplication>

using namespace csapex;

MetaPort::MetaPort(bool output, QWidget *parent)
    : Port(parent), output_(output)
{
    setContextMenuPolicy(Qt::CustomContextMenu);

    QObject::connect(this, &MetaPort::customContextMenuRequested, this, &MetaPort::showContextMenu);
}

void MetaPort::showContextMenu(const QPoint& pos)
{
    QMenu menu("Port");

    QString type = output_ ? "output" : "input";
    QAction add_port(QString("Add new ") + type, &menu);
    QObject::connect(&add_port, &QAction::triggered, this, &MetaPort::triggerCreatePort);
    menu.addAction(&add_port);

    menu.exec(QCursor::pos());
}

void MetaPort::triggerCreatePort()
{
    bool ok = false;
    QString label = QInputDialog::getText(QApplication::activeWindow(), "Label", "Enter a new label",
                                         QLineEdit::Normal, "", &ok);
    if(!ok) {
        return;
    }

    bool optional = true;

    ConnectionTypePtr type(new connection_types::AnyMessage);
    Q_EMIT createPortRequest(output_, type, label.toStdString(), optional);
}

void MetaPort::dragEnterEvent(QDragEnterEvent* e)
{
    if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION))) {
        e->acceptProposedAction();
    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
        e->acceptProposedAction();
    }
}

void MetaPort::dragMoveEvent(QDragMoveEvent* e)
{
    if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION)) ||
            e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
        e->acceptProposedAction();
    }
}

void MetaPort::dropEvent(QDropEvent* e)
{
    if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION))) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        if(from) {
            auto type = from->getType();
            auto label = from->getLabel();
            bool optional = false;

            Q_EMIT createPortAndConnectRequest(from, type, label, optional);
        }

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        if(from) {
            auto type = from->getType();
            auto label = from->getLabel();
            bool optional = false;

            Q_EMIT createPortAndMoveRequest(from, type, label, optional);
        }

    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_meta_port.cpp"

