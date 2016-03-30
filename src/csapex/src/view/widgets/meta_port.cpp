/// HEADER
#include <csapex/view/widgets/meta_port.h>

/// COMPONENT
#include <csapex/model/connectable.h>

/// SYSTEM
#include <QDragEnterEvent>
#include <QMimeData>
#include <iostream>

using namespace csapex;

MetaPort::MetaPort(QWidget *parent)
    : Port(parent)
{
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
            // TODO: make command
            std::cerr << "create port" << std::endl;
            auto type = from->getType();
            auto label = from->getLabel();
            bool optional = false;

            Q_EMIT createPortRequest(from, type, label, optional);
        }

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {

    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_meta_port.cpp"

