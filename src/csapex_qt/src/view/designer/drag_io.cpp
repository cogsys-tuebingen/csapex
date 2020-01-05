/// HEADER
#include <csapex/view/designer/drag_io.h>

/// PROJECT
#include <csapex/command/add_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/paste_graph.h>
#include <csapex/csapex_mime.h>
#include <csapex/factory/node_factory.h>
#include <csapex/factory/snippet_factory.h>
#include <csapex/model/connection.h>
#include <csapex/model/connector.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/plugin/plugin_manager.hpp>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/node/box.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <QMimeData>

using namespace csapex;

Q_DECLARE_METATYPE(ConnectorPtr)

DragIO::DragIO(PluginLocatorPtr locator, CommandExecutor* dispatcher)
  : loaded_(false), plugin_locator_(locator), manager_(new PluginManager<DragIOHandler>("csapex::DragIOHandler")), dispatcher_(dispatcher)
{
}

DragIO::~DragIO()
{
    handler_.clear();
    delete manager_;
}

void DragIO::load()
{
    if (loaded_) {
        return;
    }

    manager_->load(plugin_locator_.get());

    for (const auto& pair : manager_->getConstructors()) {
        try {
            DragIOHandler::Ptr handler(pair.second());
            registerHandler(handler);

        } catch (const std::exception& e) {
            std::cerr << "cannot load DragIOHandler " << pair.first << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
        }
    }

    loaded_ = true;
}

void DragIO::registerHandler(DragIOHandler::Ptr h)
{
    handler_.push_back(h);
}

void DragIO::dragEnterEvent(GraphView* src, QDragEnterEvent* e)
{
    if (!loaded_) {
        load();
    }

    if (e->mimeData()->hasFormat("xcsapex/node-list")) {
        e->acceptProposedAction();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::node))) {
        e->acceptProposedAction();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::snippet))) {
        e->acceptProposedAction();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create))) {
        e->acceptProposedAction();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        e->acceptProposedAction();

    } else {
        if (e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
            QByteArray itemData = e->mimeData()->data("application/x-qabstractitemmodeldatalist");
            QDataStream stream(&itemData, QIODevice::ReadOnly);

            int r, c;
            QMap<int, QVariant> v;
            stream >> r >> c >> v;

            std::string cmd = v[Qt::UserRole].toString().toStdString();

            if (cmd == csapex::mime::node || cmd == csapex::mime::snippet) {
                e->accept();
            }
        }
    }

    for (auto h : handler_) {
        if (h->handleEnter(src, dispatcher_, e)) {
            return;
        }
    }

    if (e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
        QByteArray itemData = e->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&itemData, QIODevice::ReadOnly);

        int r, c;
        QMap<int, QVariant> v;
        stream >> r >> c >> v;

        std::string cmd = v[Qt::UserRole].toString().toStdString();

        if (cmd != csapex::mime::node && cmd != csapex::mime::snippet) {
            std::cout << "warning: data is ";
            typedef const std::pair<int, QVariant> PAIR;
            for (PAIR& pair : v.toStdMap()) {
                std::cout << pair.first << ":\t" << pair.second.toString().toStdString() << '\n';
            }
            std::cout << std::endl;
        }
    }
}

void DragIO::dragMoveEvent(GraphView* src, QDragMoveEvent* e)
{
    if (!loaded_) {
        load();
    }

    if (e->mimeData()->hasFormat("xcsapex/node-list")) {
        e->acceptProposedAction();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::node))) {
        e->acceptProposedAction();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::snippet))) {
        e->acceptProposedAction();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create))) {
        if (!e->isAccepted()) {
            ConnectorPtr c = e->mimeData()->property("Connector").value<ConnectorPtr>();
            e->acceptProposedAction();

            DesignerScene* scene = src->designerScene();
            scene->deleteTemporaryConnections();
            scene->addTemporaryConnection(c, src->mapToScene(e->pos()));
        } else {
            DesignerScene* scene = src->designerScene();
            scene->update();
        }

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        if (!e->isAccepted()) {
            ConnectorPtr c = e->mimeData()->property("Connector").value<ConnectorPtr>();
            e->acceptProposedAction();

            DesignerScene* scene = src->designerScene();
            GraphFacade* graph_facade = src->getGraphFacade();

            scene->deleteTemporaryConnections();
            for (const UUID& other_id : c->getConnectedPorts()) {
                if (ConnectorPtr p = graph_facade->findConnectorNoThrow(other_id)) {
                    if (c->isOutput()) {
                        scene->addTemporaryConnection(p, src->mapToScene(e->pos()));
                    } else {
                        scene->addTemporaryConnection(p, src->mapToScene(e->pos()));
                    }
                }
            }
            scene->update();
        }

    } else if (e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
        QByteArray itemData = e->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&itemData, QIODevice::ReadOnly);

        int r, c;
        QMap<int, QVariant> v;
        stream >> r >> c >> v;

        std::string cmd = v[Qt::UserRole].toString().toStdString();

        if (cmd == csapex::mime::node || cmd == csapex::mime::snippet) {
            e->accept();
        }
    } else {
        for (auto h : handler_) {
            if (h->handleMove(src, dispatcher_, e)) {
                return;
            }
        }
    }
}

void DragIO::dropEvent(GraphView* src, QDropEvent* e, const QPointF& scene_pos)
{
    if (!loaded_) {
        load();
    }

    std::cout << "warning: drop event: " << e->mimeData()->formats().join(", ").toStdString() << std::endl;

    if (e->mimeData()->hasFormat("xcsapex/node-list")) {
        e->setDropAction(Qt::CopyAction);
        e->accept();

        QPoint offset(e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
        QPointF pos = src->mapToScene(e->pos()) + offset;

        QByteArray itemData = e->mimeData()->data("xcsapex/node-list");
        QString serialized(itemData);
        YAML::Node doc = YAML::Load(serialized.toStdString());

        pasteGraph(src, pos, doc);
    }
    if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::node))) {
        std::string type = QString(e->mimeData()->data(QString::fromStdString(csapex::mime::node))).toStdString();

        e->setDropAction(Qt::CopyAction);
        e->accept();

        QPoint offset(e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
        QPointF pos = src->mapToScene(e->pos()) + offset;

        NodeStatePtr state;
        if (!e->mimeData()->property("state").isNull()) {
            NodeStatePtr* state_ptr = (NodeStatePtr*)e->mimeData()->property("state").value<void*>();
            state = *state_ptr;
        }

        createNode(src, type, pos, state);

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::snippet))) {
        std::string type = QString(e->mimeData()->data(QString::fromStdString(csapex::mime::snippet))).toStdString();

        e->setDropAction(Qt::CopyAction);
        e->accept();

        QPoint offset(e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
        QPointF pos = src->mapToScene(e->pos()) + offset;

        if (SnippetFactoryPtr sf = src->getViewCore().getSnippetFactory()) {
            SnippetPtr snippet = sf->getSnippet(type);

            pasteGraph(src, pos, *snippet);
        }

    } else if (e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
        QByteArray itemData = e->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&itemData, QIODevice::ReadOnly);

        int r, c;
        QMap<int, QVariant> v;
        stream >> r >> c >> v;

        std::string cmd = v[Qt::UserRole].toString().toStdString();

        if (cmd == csapex::mime::node) {
            std::string type = v[Qt::UserRole + 1].toString().toStdString();

            e->setDropAction(Qt::CopyAction);
            e->accept();

            QPoint offset(e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
            QPointF pos = src->mapToScene(e->pos()) + offset;

            NodeStatePtr state;
            if (!e->mimeData()->property("state").isNull()) {
                NodeStatePtr* state_ptr = (NodeStatePtr*)e->mimeData()->property("state").value<void*>();
                state = *state_ptr;
            }

            createNode(src, type, pos, state);

        } else if (cmd == csapex::mime::snippet) {
            std::string type = v[Qt::UserRole + 1].toString().toStdString();

            e->setDropAction(Qt::CopyAction);
            e->accept();

            QPoint offset(e->mimeData()->property("ox").toInt(), e->mimeData()->property("oy").toInt());
            QPointF pos = src->mapToScene(e->pos()) + offset;

            if (SnippetFactoryPtr sf = src->getViewCore().getSnippetFactory()) {
                SnippetPtr snippet = sf->getSnippet(type);

                pasteGraph(src, pos, *snippet);
            }
        }

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create))) {
        e->ignore();
        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnections();

    } else if (e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        e->ignore();
        DesignerScene* scene = src->designerScene();
        scene->deleteTemporaryConnections();

    } else {
        for (auto h : handler_) {
            if (h->handleDrop(src, dispatcher_, e, scene_pos)) {
                return;
            }
        }
    }
}

void DragIO::createNode(GraphView* src, std::string type, const QPointF& pos, NodeStatePtr state)
{
    GraphFacade* gf = src->getGraphFacade();
    UUID uuid = gf->generateUUID(type);

    dispatcher_->executeLater(Command::Ptr(new command::AddNode(gf->getAbsoluteUUID(), type, Point(pos.x(), pos.y()), uuid, state)));
}

void DragIO::pasteGraph(GraphView* src, const QPointF& pos, const Snippet& blueprint)
{
    GraphFacade* gf = src->getGraphFacade();
    CommandPtr cmd(new command::PasteGraph(gf->getAbsoluteUUID(), blueprint, Point(pos.x(), pos.y())));

    dispatcher_->executeLater(cmd);
}
