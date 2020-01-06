#ifndef UPDATE_PARAMETER_H
#define UPDATE_PARAMETER_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/param/param_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <any>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT UpdateParameter : public CommandImplementation<UpdateParameter>
{
    COMMAND_HEADER(UpdateParameter);

public:
    explicit UpdateParameter(const UUID& parameter_uuid, const param::Parameter& param);

    bool isUndoable() const override;

    std::string getDescription() const override;

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
    param::ParameterPtr parameter_;
};

}  // namespace command

}  // namespace csapex
#endif  // UPDATE_PARAMETER_H
