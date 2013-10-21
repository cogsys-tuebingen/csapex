#ifndef CSAPEX_FWD_H
#define CSAPEX_FWD_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

#define FWD(name) \
    class name;\
    typedef boost::shared_ptr<name> name##Ptr;\
    typedef boost::shared_ptr<const name> name##ConstPtr


namespace csapex
{
class CorePlugin;

FWD(Box);
FWD(BoxGroup);
FWD(BoxedObject);

FWD(Connector);
FWD(ConnectorIn);
FWD(ConnectorOut);
FWD(Connection);

FWD(Graph);

FWD(ConnectionType);

FWD(Template);

FWD(Node);
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

class AddBox;
class DeleteBox;
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
