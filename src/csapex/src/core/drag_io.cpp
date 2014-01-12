/// HEADER
#include <csapex/core/drag_io.h>

/// PROJECT
#include <csapex/model/group.h>
#include <csapex/model/connectable.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex/manager/box_manager.h>
#include <csapex/view/overlay.h>
#include <csapex/command/add_node.h>
#include <csapex/command/dispatcher.h>

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

DragIO::Handler::Handler(CommandDispatcher* dispatcher)
    : dispatcher_(dispatcher)
{

}

void DragIO::Handler::dragEnterEvent(QWidget* src, Overlay *overlay, QDragEnterEvent* e)
{
    std::cout << "enter: " << e->format() << std::endl;
    if(e->mimeData()->hasFormat(Box::MIME)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(Box::MIME_MOVE)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
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
                BoxManager::instance().startPlacingBox(src, type, QPoint(0,0));
            }
        }
    }


    Q_FOREACH(HandlerEnter::Ptr h, DragIO::instance().handler_enter) {
        if(h->handle(dispatcher_, src, overlay, e)) {
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
            Q_FOREACH(PAIR& pair, v.toStdMap()) {
                std::cout << pair.first << ":\t" << pair.second.toString().toStdString() << '\n';
            }
            std::cout << std::endl;
        }
    }

}

void DragIO::Handler::dragMoveEvent(QWidget *src, Overlay* overlay, QDragMoveEvent* e)
{
    if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        Connectable* c = dynamic_cast<Connectable*>(e->mimeData()->parent());
        overlay->deleteTemporaryConnections();
        overlay->addTemporaryConnection(c, e->pos());
        overlay->repaint();

    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* c = dynamic_cast<Connectable*>(e->mimeData()->parent());
        overlay->deleteTemporaryConnections();

        if(c->isOutput()) {
            ConnectorOut* out = dynamic_cast<ConnectorOut*> (c);
            for(ConnectorOut::TargetIterator it = out->beginTargets(); it != out->endTargets(); ++it) {
                overlay->addTemporaryConnection(*it, e->pos());
            }
        } else {
            ConnectorIn* in = dynamic_cast<ConnectorIn*> (c);
            overlay->addTemporaryConnection(in->getSource(), e->pos());
        }
        overlay->repaint();

    } else if(e->mimeData()->hasFormat(Box::MIME_MOVE)) {
        std::string uuid_tmp = e->mimeData()->text().toStdString();
        UUID uuid = UUID::make_forced(uuid_tmp);

        Box* box = dispatcher_->getGraph()->findNode(uuid)->getBox();
        QPoint offset_value(e->mimeData()->data(Box::MIME_MOVE + "/x").toInt(),
                            e->mimeData()->data(Box::MIME_MOVE + "/y").toInt());
        box->move(e->pos() + offset_value);

        overlay->repaint();

    } else {
        Q_FOREACH(HandlerMove::Ptr h, DragIO::instance().handler_move) {
            if(h->handle(dispatcher_, src, overlay, e)) {
                return;
            }
        }
    }
}

void DragIO::Handler::dropEvent(QWidget *src, Overlay* overlay, QDropEvent* e)
{
    std::cout << "warning: drop event: " << e->mimeData()->formats().join(", ").toStdString() << std::endl;

    Graph::Ptr graph_ = dispatcher_->getGraph();

    if(e->mimeData()->hasFormat(Box::MIME)) {
        QByteArray b = e->mimeData()->data(Box::MIME);
        std::string type = (QString(b)).toStdString();

        e->setDropAction(Qt::CopyAction);
        e->accept();

        QPoint offset (e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
        QPoint pos = e->pos() + offset;


        UUID uuid = UUID::make(graph_->makeUUIDPrefix(type));
        dispatcher_->executeLater(Command::Ptr(new command::AddNode(type, pos, UUID::NONE, uuid, NodeStateNullPtr)));

    } else if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        e->ignore();
    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        e->ignore();
    } else if(e->mimeData()->hasFormat(Box::MIME_MOVE)) {
        e->acceptProposedAction();
        e->setDropAction(Qt::MoveAction);
    } else {
        Q_FOREACH(HandlerDrop::Ptr h, DragIO::instance().handler_drop) {
            if(h->handle(dispatcher_, src, overlay, e)) {
                return;
            }
        }
    }
}
