#ifndef COMMAND_META_H
#define COMMAND_META_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <vector>
#include <string>

namespace csapex
{
namespace command
{

struct Meta : public Command {
    typedef std::shared_ptr<Meta> Ptr;

    Meta(const std::string& type);
    void clear();
    void add(Command::Ptr cmd);

    int commands() const;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual QTreeWidgetItem* createDebugInformation() const;

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    std::vector<Command::Ptr> nested;
    bool locked;
    std::string type;
};
}
}


#endif // COMMAND_META_H
