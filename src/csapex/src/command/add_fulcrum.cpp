/// HEADER
#include <csapex/command/add_fulcrum.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>

using namespace csapex::command;

AddFulcrum::AddFulcrum(int connection_id, int sub_section_to_split, const QPoint &pos)
    : connection_id(connection_id), sub_section_to_split(sub_section_to_split), pos(pos)
{

}

bool AddFulcrum::execute()
{
    Graph::root()->getConnectionWithId(connection_id)->addFulcrum(sub_section_to_split, pos);
    return true;
}

bool AddFulcrum::undo()
{
    Graph::root()->getConnectionWithId(connection_id)->deleteFulcrum(sub_section_to_split);
    return true;
}

bool AddFulcrum::redo()
{
    return execute();
}

