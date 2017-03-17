/// HEADER
#include <csapex/command/quit.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph_facade.h>
#include <csapex/utility/error_handling.h>
#include <csapex/utility/assert.h>
#include <csapex/command/command_serializer.h>

/// SYSTEM
#include <sstream>
#include <iostream>
#include <typeindex>

using namespace csapex::command;


Quit::Quit()
    : Command(AUUID(UUID::NONE))
{

}

bool Quit::isUndoable() const
{
    return false;
}

std::string Quit::getType() const
{
    return "Quit";
}

std::string Quit::getDescription() const
{
    return "Quit Request";
}

bool Quit::doExecute()
{
    csapex::error_handling::stop_request()();
    return true;
}

bool Quit::doUndo()
{
    return true;
}

bool Quit::doRedo()
{
    return doExecute();
}

namespace csapex
{
namespace command
{

class QuitSerializer : public CommandSerializerInterface
{
    virtual void serialize(const CommandPtr& packet, SerializationBuffer &data) override
    {
        std::cerr << "serializing quit" << std::endl;
    }
    virtual CommandPtr deserialize(SerializationBuffer& data) override
    {
        std::cerr << "deserializing quit" << std::endl;
        return std::make_shared<Quit>();
    }
};
}
CommandSerializerRegistered<QuitSerializer> g_register_command_serializer_quit_("Quit");
}


