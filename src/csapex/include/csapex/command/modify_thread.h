#ifndef MODIFY_THREAD_H
#define MODIFY_THREAD_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT ModifyThread : public CommandImplementation<ModifyThread>
{
    COMMAND_HEADER(ModifyThread);

public:
    ModifyThread(int thread_id, const std::string& name);

    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    int id;
    std::string name;
    std::string old_name;
};

}

}

#endif // MODIFY_THREAD_H
