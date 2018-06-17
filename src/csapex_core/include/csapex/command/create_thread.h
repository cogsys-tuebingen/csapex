#ifndef CREATE_THREAD_H
#define CREATE_THREAD_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT CreateThread : public CommandImplementation<CreateThread>
{
    COMMAND_HEADER(CreateThread);

public:
    CreateThread(const AUUID &graph_uuid, const UUID& node, const std::string &name);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    std::string name;

    int old_id;
    int new_id;
};

}

}
#endif // CREATE_THREAD_H

