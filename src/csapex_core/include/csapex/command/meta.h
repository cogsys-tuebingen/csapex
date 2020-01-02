#ifndef COMMAND_META_H
#define COMMAND_META_H

/// COMPONENT
#include "command_impl.hpp"

/// SYSTEM
#include <vector>
#include <string>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT Meta : public CommandImplementation<Meta>
{
    COMMAND_HEADER(Meta);

public:
    typedef std::shared_ptr<Meta> Ptr;

    Meta(const AUUID& graph_uuid, const std::string& type, bool transaction = false);
    virtual void clear();
    void add(Command::Ptr cmd);

    int commands() const;

    void init(GraphFacadeImplementation* root, CsApexCore& core) override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;
    bool cloneData(const Meta& other) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    void accept(int level, std::function<void(int, const Command&)> callback) const override;

    std::string getDescription() const override;

protected:
    std::vector<Command::Ptr> nested;
    bool locked;
    bool transaction;
    std::string type;
};
}  // namespace command
}  // namespace csapex

#endif  // COMMAND_META_H
