#ifndef COMMAND_ADD_FULCRUM_H
#define COMMAND_ADD_FULCRUM_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/data/point.h>
#include <string>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT AddFulcrum : public CommandImplementation<AddFulcrum>
{
    COMMAND_HEADER(AddFulcrum);

public:
    AddFulcrum(const AUUID& graph_uuid, int connection_id, int sub_section_to_split, const Point& pos, int type);

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;
protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getDescription() const override;

private:
    int connection_id;
    int sub_section_to_split;
    Point pos;
    int type;
};

}

}

#endif // COMMAND_ADD_FULCRUM_H
