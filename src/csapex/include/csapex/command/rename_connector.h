#ifndef RENAME_CONNECTOR_H
#define RENAME_CONNECTOR_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT RenameConnector : public CommandImplementation<RenameConnector>
{
    COMMAND_HEADER(RenameConnector);

public:
    RenameConnector(const AUUID &graph_uuid, const UUID& connector, const std::string &new_name);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    std::string new_name_;
    std::string old_name_;
};

}

}
#endif // RENAME_CONNECTOR_H
