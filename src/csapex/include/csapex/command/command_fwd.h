#ifndef COMMAND_FWD_H
#define COMMAND_FWD_H

/// shared_ptr
#include <memory>

#define FWD(name) \
    class name;\
    typedef std::shared_ptr<name> name##Ptr;\
    typedef std::unique_ptr<name> name##UniquePtr;\
    typedef std::weak_ptr<name> name##WeakPtr;\
    typedef std::shared_ptr<const name> name##ConstPtr;


namespace csapex
{

FWD(Command);
FWD(CommandDispatcher);

namespace command
{
class Meta;

class AddNode;
class DeleteNode;
class MoveBox;

class AddConnection;
class MoveConnection;
class DeleteConnection;

class AddFulcrum;
class MoveFulcrum;
class DeleteFulcrum;
class ModifyFulcrum;
}

}

#undef FWD

#endif // COMMAND_FWD_H

