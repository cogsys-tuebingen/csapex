/// HEADER
#include <csapex/command/modify_fulcrum.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/fulcrum.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <sstream>

using namespace csapex::command;

ModifyFulcrum::ModifyFulcrum(int connection_id, int fulcrum_id,
                             int f_type, const QPointF &f_handle_in, const QPointF &f_handle_out,
                             int t_type, const QPointF &t_handle_in, const QPointF &t_handle_out)
    : connection_id(connection_id), fulcrum_id(fulcrum_id),
      f_type(f_type), f_in(f_handle_in), f_out(f_handle_out),
      t_type(t_type), t_in(t_handle_in), t_out(t_handle_out)
{
}

std::string ModifyFulcrum::getType() const
{
    return "ModifyFulcrum";
}

std::string ModifyFulcrum::getDescription() const
{
    std::stringstream ss;
    ss << "modified fulcrum " << fulcrum_id << " of connection " << connection_id;
    ss << "(type=" << t_type << ", in: " << t_in.x() << "/" << t_in.y() << ", out: " << t_out.x() << "/" << t_out.y() << ")";
    return ss.str();
}

bool ModifyFulcrum::doExecute()
{
    graph_->getConnectionWithId(connection_id)->modifyFulcrum(fulcrum_id, t_type, t_in, t_out);
    return true;
}

bool ModifyFulcrum::doUndo()
{
    graph_->getConnectionWithId(connection_id)->modifyFulcrum(fulcrum_id, f_type, f_in, f_out);
    return true;
}

bool ModifyFulcrum::doRedo()
{
    return doExecute();
}

