#ifndef PLAYBACK_COMMAND_H
#define PLAYBACK_COMMAND_H

/// COMPONENT
#include <csapex/command/meta.h>

namespace csapex
{

namespace command
{

class CSAPEX_COMMAND_EXPORT PlaybackCommand : public Meta
{
public:
    PlaybackCommand(const AUUID& graph_uuid, const std::string& type);

    void execute(CommandPtr subcommand);

protected:
    bool doExecute() override;
};

}

}


#endif // PLAYBACK_COMMAND_H
