#ifndef CSAPEX_FWD_H
#define CSAPEX_FWD_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

#define FWD(name) \
    class name;\
    typedef boost::shared_ptr<name> name##Ptr;\
    typedef boost::shared_ptr<const name> name##ConstPtr;\
    static const name##Ptr name##NullPtr


template <typename T>
class PluginManager;

namespace csapex
{
class CorePlugin;

FWD(Box);
FWD(Group);
FWD(BoxedObject);
FWD(NodeConstructor);

FWD(Port);
FWD(Connectable);
FWD(ConnectorIn);
FWD(ConnectorOut);
FWD(Connection);

FWD(Graph);
FWD(GenericState);

FWD(ConnectionType);

FWD(Template);

FWD(Memento);
FWD(MessageProvider);

FWD(Node);
FWD(NodeState);
FWD(NodeWorker);
FWD(NodeAdapter);

class Designer;
class DesignBoard;

class DragIO;

class Overlay;

class Tag;

FWD(Command);

FWD(CommandDispatcher);

class ProfilingWidget;

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

class AddConnector;
}


}

#undef FWD

#endif // CSAPEX_FWD_H
