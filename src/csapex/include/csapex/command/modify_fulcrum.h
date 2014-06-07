#ifndef MODIFY_FULCRUM_H
#define MODIFY_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{
namespace command
{

struct ModifyFulcrum : public Command
{
    ModifyFulcrum(int connection_id, int fulcrum_id,
                  int f_type, const QPointF& f_handle_in, const QPointF& f_handle_out,
                  int t_type, const QPointF &t_handle_in, const QPointF &t_handle_out);

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    int connection_id;
    int fulcrum_id;

    int f_type;
    QPointF f_in;
    QPointF f_out;

    int t_type;
    QPointF t_in;
    QPointF t_out;
};

}

}

#endif // MODIFY_FULCRUM_H
