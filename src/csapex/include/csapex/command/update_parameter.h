#ifndef UPDATE_PARAMETER_H
#define UPDATE_PARAMETER_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT UpdateParameter : public Command
{
    template <typename T>
    explicit UpdateParameter(const UUID &parameter_uuid, const T& value)
        : Command(parameter_uuid.getAbsoluteUUID()), uuid(parameter_uuid.getAbsoluteUUID()),
          value(value)
    {
        apex_assert(!parameter_uuid.empty());
    }

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

    virtual bool isUndoable() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    AUUID uuid;

    boost::any value;
};

}

}
#endif // UPDATE_PARAMETER_H
