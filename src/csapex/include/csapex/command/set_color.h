#ifndef SET_COLOR_H
#define SET_COLOR_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT SetColor : public Command
{
    SetColor(const AUUID &graph_uuid, const UUID& node,
             int r, int g, int b);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;

    int r;
    int g;
    int b;

    int r_orig;
    int g_orig;
    int b_orig;
};

}

}
#endif // SET_COLOR_H
