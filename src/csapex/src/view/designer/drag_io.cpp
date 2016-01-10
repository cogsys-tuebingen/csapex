/// HEADER
#include <csapex/view/designer/drag_io.h>

/// PROJECT
#include <csapex/command/add_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/node.h>
#include <csapex/view/node/box.h>
#include <csapex/view/designer/widget_controller.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/designer/designer_view.h>
#include <csapex/plugin/plugin_manager.hpp>

/// SYSTEM
#include <QMimeData>

using namespace csapex;

DragIO::DragIO(PluginLocatorPtr locator, Graph *graph, CommandDispatcher* dispatcher, WidgetControllerPtr widget_ctrl)
    : loaded_(false), plugin_locator_(locator), manager_(new PluginManager<DragIOHandler>("csapex::DragIOHandler")),
      graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl)
{
}

DragIO::~DragIO()
{
    handler_.clear();
    delete manager_;
}

void DragIO::load()
{
    if(loaded_) {
        return;
    }

    manager_->load(plugin_locator_.get());

    for(const auto& pair : manager_->getConstructors()) {
        try {
            DragIOHandler::Ptr handler(pair.second());
            registerHandler(handler);

        } catch(const std::exception& e) {
            std::cerr << "cannot load DragIOHandler " << pair.first << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
        }
    }

    loaded_ = true;
}

void DragIO::registerHandler(DragIOHandler::Ptr h)
{
    handler_.push_back(h);
}

void DragIO::dragEnterEvent(DesignerView* src, QDragEnterEvent* e)
{
    if(!loaded_) {
        load();
    }

    if(e->mimeData()->hasFormat(NodeBox::MIME)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION))) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
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
                widget_ctrl_->startPlacingBox(src, type, nullptr, QPoint(0,0));
            }
        }
    }


    for(auto h : handler_) {
        if(h->handleEnter(dispatcher_, src, e)) {
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
            for(PAIR& pair : v.toStdMap()) {
                std::cout << pair.first << ":\t" << pair.second.toString().toStdString() << '\n';
            }
            std::cout << std::endl;
        }
    }

}

void DragIO::dragMoveEvent(DesignerView *src, QDragMoveEvent* e)
{
    if(!loaded_) {
        load();
    }

    if(e->mimeData()->hasFormat(NodeBox::MIME)) {
        e->acceptProposedAction();

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION))) {
        Connectable* c = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        e->acceptProposedAction();

        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnections();
        scene->addTemporaryConnection(c, src->mapToScene(e->pos()));

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
        Connectable* c = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        e->acceptProposedAction();

        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnections();

        if(c->isOutput()) {
            Output* out = dynamic_cast<Output*> (c);
            if(out) {
                for(ConnectionPtr c : out->getConnections()) {
                    if(!c) {
                        continue;
                    }
                    Input* input = dynamic_cast<Input*>(c->to());
                    if(input) {
                        scene->addTemporaryConnection(input, src->mapToScene(e->pos()));
                    }
                }
            } else {
                Trigger* trigger = dynamic_cast<Trigger*> (c);
                if(trigger) {
                    for(Slot* slot : trigger->getTargets()) {
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
                    for(Trigger* trigger : slot->getSources()) {
                        scene->addTemporaryConnection(trigger, src->mapToScene(e->pos()));
                    }
                }
            }
        }
        scene->update();

    } else {
        for(auto h : handler_) {
            if(h->handleMove(dispatcher_, src, e)) {
                return;
            }
        }
    }
}

void DragIO::dropEvent(DesignerView *src, QDropEvent* e, const QPointF& scene_pos)
{
    if(!loaded_) {
        load();
    }

    std::cout << "warning: drop event: " << e->mimeData()->formats().join(", ").toStdString() << std::endl;

    if(e->mimeData()->hasFormat(NodeBox::MIME)) {
        QByteArray b = e->mimeData()->data(NodeBox::MIME);
        std::string type = (QString(b)).toStdString();

        e->setDropAction(Qt::CopyAction);
        e->accept();

        QPoint offset (e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
        QPointF pos = src->mapToScene(e->pos()) + offset;

        UUID uuid = graph_->generateUUID(type);

        NodeStatePtr state;

        if(!e->mimeData()->property("state").isNull()) {
            NodeStatePtr* state_ptr = (NodeStatePtr *) e->mimeData()->property("state").value<void *>();
            state = *state_ptr;
        }

        dispatcher_->executeLater(Command::Ptr(new command::AddNode(type, Point(pos.x(), pos.y()), UUID::NONE, uuid, state)));

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_CREATE_CONNECTION))) {
        e->ignore();
        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnectionsAndRepaint();

    } else if(e->mimeData()->hasFormat(QString::fromStdString(Connectable::MIME_MOVE_CONNECTIONS))) {
        e->ignore();
        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnectionsAndRepaint();

    } else {
        for(auto h : handler_) {
            if(h->handleDrop(dispatcher_, src, e, scene_pos)) {
                return;
            }
        }
    }
}
