/// HEADER
#include <csapex/drag_io.h>

/// PROJECT
#include <csapex/box.h>
#include <csapex/box_group.h>
#include <csapex/connector.h>
#include <csapex/connector_out.h>
#include <csapex/connector_in.h>
#include <csapex/box_manager.h>
#include <csapex/overlay.h>

#include <csapex/command_add_box.h>
#include <csapex/command_dispatcher.h>

using namespace csapex;

void DragIO::registerEnterHandler(HandlerEnter::Ptr h)
{
    handler_enter.push_back(h);
}
void DragIO::registerMoveHandler(HandlerMove::Ptr h)
{
    handler_move.push_back(h);
}
void DragIO::registerDropHandler(HandlerDrop::Ptr h)
{
    handler_drop.push_back(h);
}

void DragIO::dragEnterEvent(QWidget* src, Overlay *overlay, QDragEnterEvent* e)
{
    if(e->mimeData()->hasFormat(Box::MIME)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(Box::MIME_MOVE)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(Connector::MIME_CREATE)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(Connector::MIME_MOVE)) {
        e->acceptProposedAction();

    } else {
        if(e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
            QByteArray itemData = e->mimeData()->data("application/x-qabstractitemmodeldatalist");
            QDataStream stream(&itemData, QIODevice::ReadOnly);

            int r, c;
            QMap<int, QVariant> v;
            stream >> r >> c >> v;

            std::string cmd = v[Qt::UserRole].toString().toStdString();

            if(cmd == Box::MIME.toStdString()) {
                e->accept();

                std::string type = v[Qt::UserRole+1].toString().toStdString();
                if(type == BoxGroup::MIME.toStdString()) {
                    BoxManager::instance().startPlacingMetaBox(src, QPoint(0,0));
                } else {
                    BoxManager::instance().startPlacingBox(src, type, QPoint(0,0));
                }

            }
        }
    }


    foreach(HandlerEnter::Ptr h, handler_enter) {
        if(h->handle(src, overlay, e)) {
            return;
        }
    }

    if(e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
        QByteArray itemData = e->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&itemData, QIODevice::ReadOnly);

        int r, c;
        QMap<int, QVariant> v;
        stream >> r >> c >> v;

        std::string cmd = v[Qt::UserRole].toString().toStdString();

        if(cmd != Box::MIME.toStdString()) {
            std::cout << "warning: data is ";
            typedef const std::pair<int, QVariant> PAIR;
            foreach(PAIR& pair, v.toStdMap()) {
                std::cout << pair.first << ":\t" << pair.second.toString().toStdString() << '\n';
            }
            std::cout << std::endl;
        }
    }

}

void DragIO::dragMoveEvent(QWidget *src, Overlay* overlay, QDragMoveEvent* e)
{
    if(e->mimeData()->hasFormat(Connector::MIME_CREATE)) {
        Connector* c = dynamic_cast<Connector*>(e->mimeData()->parent());
        overlay->deleteTemporaryConnections();
        overlay->addTemporaryConnection(c, e->pos());
        overlay->repaint();

    } else if(e->mimeData()->hasFormat(Connector::MIME_MOVE)) {
        Connector* c = dynamic_cast<Connector*>(e->mimeData()->parent());
        overlay->deleteTemporaryConnections();

        ConnectorOut* out = dynamic_cast<ConnectorOut*> (c);
        if(out) {
            for(ConnectorOut::TargetIterator it = out->beginTargets(); it != out->endTargets(); ++it) {
                overlay->addTemporaryConnection(*it, e->pos());
            }
        } else {
            ConnectorIn* in = dynamic_cast<ConnectorIn*> (c);
            overlay->addTemporaryConnection(in->getConnected(), e->pos());
        }
        overlay->repaint();

    } else if(e->mimeData()->hasFormat(Box::MIME_MOVE)) {
        std::string uuid = e->mimeData()->text().toStdString();
        Box::Ptr box = Graph::root()->findBox(uuid);
        QPoint offset_value(e->mimeData()->data(Box::MIME_MOVE + "/x").toInt(),
                            e->mimeData()->data(Box::MIME_MOVE + "/y").toInt());
        box->move(e->pos() + offset_value);

        overlay->repaint();

    } else {
        foreach(HandlerMove::Ptr h, handler_move) {
            if(h->handle(src, overlay, e)) {
                return;
            }
        }
    }
}

void DragIO::dropEvent(QWidget *src, Overlay* overlay, QDropEvent* e)
{
    std::cout << "warning: drop event: " << e->mimeData()->formats().join(", ").toStdString() << std::endl;

    Graph::Ptr graph_ = Graph::root();

    if(e->mimeData()->hasFormat(Box::MIME)) {
        QByteArray b = e->mimeData()->data(Box::MIME);
        SelectorProxy::Ptr* selectorptr = (SelectorProxy::Ptr*)b.toULongLong();

        if(!selectorptr) {
            return;
        }

        SelectorProxy::Ptr selector = *selectorptr;

        e->setDropAction(Qt::CopyAction);
        e->accept();

        QPoint offset (e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
        QPoint pos = e->pos() + offset;

        Command::Ptr add_box(new command::AddBox(selector, pos, "", graph_->makeUUID(selector->getType())));
        CommandDispatcher::execute(add_box);

    } else if(e->mimeData()->hasFormat(Connector::MIME_CREATE)) {
        e->ignore();
    } else if(e->mimeData()->hasFormat(Connector::MIME_MOVE)) {
        e->ignore();
    } else if(e->mimeData()->hasFormat(Box::MIME_MOVE)) {
        e->acceptProposedAction();
        e->setDropAction(Qt::MoveAction);
    } else {
        foreach(HandlerDrop::Ptr h, handler_drop) {
            if(h->handle(src, overlay, e)) {
                return;
            }
        }
    }
}
