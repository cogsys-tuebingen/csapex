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
    explicit UpdateParameter(const UUID& parameter_uuid, const T& value) : CommandImplementation(parameter_uuid.getAbsoluteUUID()), uuid(parameter_uuid.getAbsoluteUUID()), value(value)
    {
        if (parameter_uuid.global()) {
            apex_assert_hard(!parameter_uuid.globalName().empty());
        } else {
            apex_assert(!parameter_uuid.empty());
        }
    }

    virtual bool isUndoable() const override;

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

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

}  // namespace command

}  // namespace csapex
#endif  // UPDATE_PARAMETER_H
