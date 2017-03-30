#ifndef COMMAND_ADD_CONNECTION_HPP
#define COMMAND_ADD_CONNECTION_HPP

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{


namespace command
{

class CSAPEX_COMMAND_EXPORT AddConnection : public CommandImplementation<AddConnection>
{
    COMMAND_HEADER(AddConnection);

public:
    AddConnection(const AUUID& graph_uuid, const UUID &from_uuid, const UUID &to_uuid, bool active);

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

protected:
    bool doUndo() override;
    bool doRedo() override;

    bool doExecute() override;

    virtual std::string getDescription() const override;

protected:
    UUID from_uuid;
    UUID to_uuid;

private:
    bool active;
};
}
}
#endif // COMMAND_ADD_CONNECTION_HPP
