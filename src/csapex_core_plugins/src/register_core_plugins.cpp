/// HEADER
#include "register_core_plugins.h"

/// COMPONENT
#include <csapex_core_plugins/string_message.h>
#include <csapex_core_plugins/ros_handler.h>
#include "import_ros.h"
#include "file_importer.h"

/// PROJECT
#include <csapex/manager/connection_type_manager.h>
#include <csapex/model/tag.h>
#include <csapex/core/drag_io.h>
#include <csapex/command/add_box.h>
#include <csapex/command/dispatcher.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/box.h>

/// SYSTEM
#include <boost/regex.hpp>
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QUrl>

PLUGINLIB_EXPORT_CLASS(csapex::RegisterCorePlugins, csapex::CorePlugin)

using namespace csapex;

namespace {
class RosHandler
        : public DragIO::HandlerEnter, public DragIO::HandlerMove, public DragIO::HandlerDrop
{
    static const boost::regex fmt;

    std::string getCmd(QDropEvent* e)
    {
        QByteArray itemData = e->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&itemData, QIODevice::ReadOnly);

        int r, c;
        QMap<int, QVariant> v;
        stream >> r >> c >> v;

        return v[Qt::UserRole].toString().toStdString();
    }

    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDragEnterEvent* e) {
        if(e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
            std::string cmd = getCmd(e);

            bool cmd_could_be_topic = boost::regex_match(cmd, fmt);

            if(cmd_could_be_topic) {
                if(ROSHandler::instance().isConnected() && ROSHandler::instance().topicExists(cmd)) {
                    e->accept();
                    return true;
                }
            }
        }
        return false;
    }
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDragMoveEvent* e){
        return false;
    }
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDropEvent* e) {
        if(e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
            std::string cmd = getCmd(e);

            bool cmd_could_be_topic = boost::regex_match(cmd, fmt);

            if(cmd_could_be_topic) {
                if(ROSHandler::instance().topicExists(cmd)) {
                    QPoint pos = e->pos();

                    std::string uuid = dispatcher->getGraph()->makeUUID("csapex::ImportRos");

                    Memento::Ptr state;
                    {
                        Box::Ptr tmp = BoxManager::instance().makeBox("csapex::ImportRos", uuid);
                        assert(tmp);

                        boost::shared_ptr<ImportRos> inst = boost::dynamic_pointer_cast<ImportRos> (tmp->getContent());
                        assert(inst);

                        inst->changeTopic(cmd.c_str());

                        state = tmp->getState();
                    }

                    std::string type("csapex::ImportRos");
                    dispatcher->execute(Command::Ptr(new command::AddBox(type, pos, "", uuid, state)));

                    return true;
                }
            }
        }
        return false;
    }
};

const boost::regex RosHandler::fmt("[a-zA-Z_\\-/]+");



class FileHandler
        : public DragIO::HandlerEnter, public DragIO::HandlerMove, public DragIO::HandlerDrop
{
    static const std::string format;

    QList<QUrl> getFiles(QDropEvent* e)
    {
        QByteArray itemData = e->mimeData()->data(format.c_str());
        QDataStream stream(&itemData, QIODevice::ReadOnly);

        int r, c;
        QMap<int, QVariant> v;
        stream >> r >> c >> v;

        e->accept();

        return e->mimeData()->urls();
    }

    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDragEnterEvent* e) {
        if(e->mimeData()->hasFormat(format.c_str())) {
            QList<QUrl> files = getFiles(e);

            if(files.size() != 0) {
                e->accept();
                return true;
            }
        }
        return false;
    }
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDragMoveEvent* e){
        return false;
    }
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDropEvent* e) {
        if(e->mimeData()->hasFormat(format.c_str())) {
            QList<QUrl> files = getFiles(e);

            if(files.size() == 0) {
                return false;
            }

            if(files.size() > 1) {
                std::cerr << "warning: droppend more than one file, using the first one" << std::endl;
            }

            std::cout << "file: " << files.first().toString().toStdString() << std::endl;
            QFile file(files.first().toLocalFile());
            if(file.exists()) {
                QPoint pos = e->pos();

                std::string uuid = dispatcher->getGraph()->makeUUID("csapex::FileImporter");

                Memento::Ptr state;
                {
                    Box::Ptr tmp = BoxManager::instance().makeBox("csapex::FileImporter", uuid);
                    assert(tmp);

                    boost::shared_ptr<FileImporter> inst = boost::dynamic_pointer_cast<FileImporter> (tmp->getContent());
                    assert(inst);

                    // TODO: allow multiple files
                    inst->import(files.first().toString());

                    state = tmp->getState();
                }

                std::string type("csapex::FileImporter");
                dispatcher->execute(Command::Ptr(new command::AddBox(type, pos, "", uuid, state)));

                e->accept();
                return true;
            }
        }
        return false;
    }
};

const std::string FileHandler::format = "text/uri-list";

}

RegisterCorePlugins::RegisterCorePlugins()
{
}

void RegisterCorePlugins::init()
{
    Tag::createIfNotExists("Buffer");
    Tag::createIfNotExists("General");
    Tag::createIfNotExists("Input");
    Tag::createIfNotExists("Output");
    Tag::createIfNotExists("RosIO");
    Tag::createIfNotExists("ConsoleIO");

    DragIO::registerHandler<RosHandler>();
    DragIO::registerHandler<FileHandler>();

    ConnectionTypeManager::registerMessage<connection_types::StringMessage>("std::string");
}

void RegisterCorePlugins::shutdown()
{
    ROSHandler::instance().stop();
}
