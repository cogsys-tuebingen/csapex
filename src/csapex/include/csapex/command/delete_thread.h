#ifndef DELETE_THREAD_H
#define DELETE_THREAD_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT DeleteThread : public CommandImplementation<DeleteThread>
{
    COMMAND_HEADER(DeleteThread);

public:
    DeleteThread(int thread_id);

    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    int id;
    std::string name;
};

}

}

#endif // DELETE_THREAD_H
