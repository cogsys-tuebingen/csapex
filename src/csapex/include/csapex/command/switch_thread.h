#ifndef SWITCH_THREAD_H
#define SWITCH_THREAD_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT SwitchThread : public CommandImplementation<SwitchThread>
{
    COMMAND_HEADER(SwitchThread);

public:
    SwitchThread(const AUUID& graph_uuid, const UUID& node, int thread_id);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    int old_id;
    int id;
    std::string name;
};

}

}
#endif // SWITCH_THREAD_H

