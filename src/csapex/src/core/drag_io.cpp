/// HEADER
#include <csapex/core/drag_io.h>

/// PROJECT
#include <csapex/command/add_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/node.h>
#include <csapex/view/box.h>
#include <csapex/view/widget_controller.h>
#include <csapex/view/designer_scene.h>
#include <csapex/view/designer_view.h>

/// SYSTEM
#include <QMimeData>

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

DragIO::DragIO(Graph *graph, CommandDispatcher* dispatcher, WidgetControllerPtr widget_ctrl)
    : graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl)
{

}

void DragIO::dragEnterEvent(DesignerView* src, QDragEnterEvent* e)
{
    if(e->mimeData()->hasFormat(NodeBox::MIME)) {
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

            if(cmd == NodeBox::MIME.toStdString()) {
                e->accept();

                std::string type = v[Qt::UserRole+1].toString().toStdString();
                widget_ctrl_->startPlacingBox(src, type, NodeStateNullPtr, QPoint(0,0));
            }
        }
    }


    Q_FOREACH(HandlerEnter::Ptr h, handler_enter) {
        if(h->handle(dispatcher_, src, e)) {
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

        if(cmd != NodeBox::MIME.toStdString()) {
            std::cout << "warning: data is ";
            typedef const std::pair<int, QVariant> PAIR;
            Q_FOREACH(PAIR& pair, v.toStdMap()) {
                std::cout << pair.first << ":\t" << pair.second.toString().toStdString() << '\n';
            }
            std::cout << std::endl;
        }
    }

}

void DragIO::dragMoveEvent(DesignerView *src, QDragMoveEvent* e)
{
    if(e->mimeData()->hasFormat(NodeBox::MIME)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        Connectable* c = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        e->acceptProposedAction();

        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnections();
        scene->addTemporaryConnection(c, src->mapToScene(e->pos()));

    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        Connectable* c = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        e->acceptProposedAction();

        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnections();

        if(c->isOutput()) {
            Output* out = dynamic_cast<Output*> (c);
            if(out) {
                foreach(Input* input, out->getTargets()) {
                    scene->addTemporaryConnection(input, src->mapToScene(e->pos()));
                }
            } else {
                Trigger* trigger = dynamic_cast<Trigger*> (c);
                if(trigger) {
                    foreach(Slot* slot, trigger->getTargets()) {
                        scene->addTemporaryConnection(slot, src->mapToScene(e->pos()));
                    }
                }
            }
        } else {
            Input* in = dynamic_cast<Input*> (c);
            if(in) {
                scene->addTemporaryConnection(in->getSource(), src->mapToScene(e->pos()));
            } else {
                Slot* slot = dynamic_cast<Slot*> (c);
                if(slot) {
                    scene->addTemporaryConnection(slot->getSource(), src->mapToScene(e->pos()));
                }
            }
        }
        scene->update();

    } else {
        Q_FOREACH(HandlerMove::Ptr h, handler_move) {
            if(h->handle(dispatcher_, src, e)) {
                return;
            }
        }
    }
}

void DragIO::dropEvent(DesignerView *src, QDropEvent* e, const QPointF& scene_pos)
{
    std::cout << "warning: drop event: " << e->mimeData()->formats().join(", ").toStdString() << std::endl;

    if(e->mimeData()->hasFormat(NodeBox::MIME)) {
        QByteArray b = e->mimeData()->data(NodeBox::MIME);
        std::string type = (QString(b)).toStdString();

        e->setDropAction(Qt::CopyAction);
        e->accept();

        QPoint offset (e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
        QPointF pos = src->mapToScene(e->pos()) + offset;


        UUID uuid = UUID::make(graph_->makeUUIDPrefix(type));

        NodeStatePtr state;

        if(!e->mimeData()->property("state").isNull()) {
            NodeStatePtr* state_ptr = (NodeStatePtr *) e->mimeData()->property("state").value<void *>();
            state = *state_ptr;
        }

        dispatcher_->executeLater(Command::Ptr(new command::AddNode(type, pos.toPoint(), UUID::NONE, uuid, state)));

    } else if(e->mimeData()->hasFormat(Connectable::MIME_CREATE_CONNECTION)) {
        e->ignore();
        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnectionsAndRepaint();

    } else if(e->mimeData()->hasFormat(Connectable::MIME_MOVE_CONNECTIONS)) {
        e->ignore();
        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnectionsAndRepaint();

    } else {
        Q_FOREACH(HandlerDrop::Ptr h, handler_drop) {
            if(h->handle(dispatcher_, src, e, scene_pos)) {
                return;
            }
        }
    }
}
