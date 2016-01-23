#ifndef COMMAND_META_H
#define COMMAND_META_H

/// COMPONENT
#include "command.h"

/// SYSTEM
#include <vector>
#include <string>

namespace csapex
{
namespace command
{

class Meta : public Command
{
public:
    typedef std::shared_ptr<Meta> Ptr;

    Meta(const AUUID& graph_uuid, const std::string& type);
    void clear();
    void add(Command::Ptr cmd);

    int commands() const;

    virtual void init(Settings* settings, GraphFacade* graph_facade,
                      ThreadPool* thread_pool, NodeFactory *node_factory) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual void accept(int level, std::function<void (int, const Command &)> callback) const override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    std::vector<Command::Ptr> nested;
    bool locked;
    std::string type;
};
}
}


#endif // COMMAND_META_H
