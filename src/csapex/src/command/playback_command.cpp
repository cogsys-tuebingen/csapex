/// HEADER
#include <csapex/command/playback_command.h>

using namespace csapex;
using namespace command;

PlaybackCommand::PlaybackCommand(const AUUID &graph_uuid, const std::string &type)
    : Meta(graph_uuid, type)
{

}

void PlaybackCommand::execute(CommandPtr subcommand)
{
    Command::executeCommand(subcommand);

    add(subcommand);
}

bool PlaybackCommand::doExecute()
{
    return true;
}
