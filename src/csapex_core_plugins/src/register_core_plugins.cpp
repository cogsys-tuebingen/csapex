/// HEADER
#include "register_core_plugins.h"

/// COMPONENT
#include <csapex_core_plugins/string_message.h>
#include <csapex_core_plugins/ros_handler.h>
#include "import_ros.h"

/// PROJECT
#include <csapex/connection_type_manager.h>
#include <csapex/tag.h>
#include <csapex/drag_io.h>
#include <csapex/command_add_box.h>
#include <csapex/command_dispatcher.h>
#include <csapex/box_manager.h>
#include <csapex/box.h>

/// SYSTEM
#include <boost/regex.hpp>
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>
#include <QStringList>

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

    virtual bool handle(QWidget *src, Overlay* overlay, QDragEnterEvent* e) {
        if(e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
            std::string cmd = getCmd(e);

            bool cmd_could_be_topic = boost::regex_match(cmd, fmt);

            if(cmd_could_be_topic) {
                if(ROSHandler::instance().topicExists(cmd)) {
                    e->accept();
                    return true;
                }
            }
        }
        return false;
    }
    virtual bool handle(QWidget *src, Overlay* overlay, QDragMoveEvent* e){
        return false;
    }
    virtual bool handle(QWidget *src, Overlay* overlay, QDropEvent* e) {
        if(e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
            std::string cmd = getCmd(e);

            bool cmd_could_be_topic = boost::regex_match(cmd, fmt);

            if(cmd_could_be_topic) {
                if(ROSHandler::instance().topicExists(cmd)) {
                    QPoint pos = e->pos();

                    Memento::Ptr state;
                    {
                        Box::Ptr tmp = BoxManager::instance().makeBox(QPoint(), "csapex::ImportRos");
                        assert(tmp);

                        boost::shared_ptr<ImportRos> inst = boost::dynamic_pointer_cast<ImportRos> (tmp->getContent());
                        assert(inst);

                        inst->changeTopic(cmd.c_str());

                        state = tmp->getState();
                    }

                    CommandDispatcher::execute(Command::Ptr(new command::AddBox(BoxManager::instance().getSelector("csapex::ImportRos"), pos, state)));

                    return true;
                }
            }
        }
        return false;
    }
};

const boost::regex RosHandler::fmt("[a-zA-Z_\\-/]+");
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

    boost::shared_ptr<RosHandler> handler(new RosHandler);

    DragIO::registerEnterHandler(handler);
    DragIO::registerMoveHandler(handler);
    DragIO::registerDropHandler(handler);

    ConnectionTypeManager::registerMessage("std::string", boost::bind(&connection_types::StringMessage::make));
}
