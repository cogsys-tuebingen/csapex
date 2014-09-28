#ifndef CSAPEX_FWD_H
#define CSAPEX_FWD_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

/// FORWARD DELCARATION
namespace boost {
class mutex;
}


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

FWD(NodeBox);
FWD(Group);
FWD(NodeConstructor);
FWD(NodeStatistics);

FWD(ThreadPool);

FWD(Port);
FWD(Connectable);
FWD(Input);
FWD(Output);
FWD(Connection);
FWD(Fulcrum);

FWD(Graph);
FWD(GraphWorker);
FWD(GenericState);

FWD(ConnectionType);

FWD(Template);

FWD(Memento);
FWD(MessageProvider);

FWD(Node);
FWD(NodeState);
FWD(NodeWorker);
FWD(NodeAdapter);
FWD(NodeModifier);

FWD(WidgetController);

class StreamRelay;

class Designer;
class DesignBoard;
class DesignerScene;
class DesignerView;

class MovableGraphicsProxyWidget;

class DragIO;

class Overlay;

FWD(NodeFactory);
FWD(NodeAdapterFactory);

FWD(Timable);

FWD(Tag);

class Settings;

FWD(Buffer);

FWD(Timer);

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
class ModifyFulcrum;

class AddConnector;
}


}

#undef FWD

#endif // CSAPEX_FWD_H
