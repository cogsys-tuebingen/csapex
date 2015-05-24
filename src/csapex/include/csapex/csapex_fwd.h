#ifndef CSAPEX_FWD_H
#define CSAPEX_FWD_H

/// SYSTEM
#include <memory>

/// FORWARD DELCARATION
namespace std
{
class mutex;
}


#define FWD(name) \
    class name;\
    typedef std::shared_ptr<name> name##Ptr;\
    typedef std::unique_ptr<name> name##UniquePtr;\
    typedef std::weak_ptr<name> name##WeakPtr;\
    typedef std::shared_ptr<const name> name##ConstPtr;


namespace csapex
{
template <typename T>
class PluginManager;

FWD(CsApexCore);
FWD(CorePlugin);
FWD(BootstrapPlugin);
FWD(PluginLocator);

FWD(NodeBox);
FWD(Group);
FWD(NodeConstructor);
FWD(NodeStatistics);

FWD(ThreadPool);
FWD(ThreadGroup);
FWD(Scheduler);
FWD(Schedulable);

FWD(Port);
FWD(Connectable);
FWD(Input);
FWD(Output);
FWD(InputTransition);
FWD(OutputTransition);
FWD(Connection);
FWD(Fulcrum);
FWD(Trigger);
FWD(Slot);

FWD(Graph);
FWD(GraphWorker);
FWD(GenericState);

FWD(ConnectionType);

FWD(Template);

FWD(Memento);

FWD(MessageProvider);
FWD(MessageRenderer);

FWD(Node);
FWD(NodeState);
FWD(NodeWorker);
FWD(NodeAdapter);
FWD(NodeModifier);
FWD(NodeAdapterBuilder);

FWD(WidgetController);

class StreamRelay;

class Designer;
class DesignBoard;
class DesignerScene;
class DesignerView;
class MinimapWidget;

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
FWD(ActivityTimeline);
FWD(ActivityLegend);

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

namespace connection_types
{
FWD(Message);
FWD(Signal);

template <typename Type>
struct GenericPointerMessage;
template <typename Type>
struct GenericValueMessage;
}

}

#undef FWD

#endif // CSAPEX_FWD_H
