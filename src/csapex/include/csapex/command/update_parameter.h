#ifndef UPDATE_PARAMETER_H
#define UPDATE_PARAMETER_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT UpdateParameter : public CommandImplementation<UpdateParameter>
{
    COMMAND_HEADER(UpdateParameter);

public:
    template <typename T>
    explicit UpdateParameter(const UUID &parameter_uuid, const T& value)
        : CommandImplementation(parameter_uuid.getAbsoluteUUID()), uuid(parameter_uuid.getAbsoluteUUID()),
          value(value)
    {
        apex_assert(!parameter_uuid.empty());
    }

    virtual std::string getDescription() const override;

    virtual bool isUndoable() const override;

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

    virtual void cloneFrom(const Command& other);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    template <typename T>
    void setParameter(const T& value);

private:
    AUUID uuid;

    boost::any value;
};

}

}
#endif // UPDATE_PARAMETER_H
