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
    ModifyThread(int thread_id, const std::string& name, std::vector<bool> cpu_affinity);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    ThreadGroup* getGroup();

private:
    int id;

    std::string name;
    std::vector<bool> affinity;

    // cache
    std::string old_name;
    std::vector<bool> old_affinity;
};

}  // namespace command

}  // namespace csapex

#endif  // MODIFY_THREAD_H
