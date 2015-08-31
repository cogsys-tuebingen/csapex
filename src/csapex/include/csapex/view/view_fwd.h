#ifndef VIEW_FWD_H
#define VIEW_FWD_H

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
FWD(ActivityTimeline);
FWD(ActivityLegend);
FWD(NodeBox);
FWD(MessageRenderer);
FWD(NodeAdapter);
FWD(NodeAdapterBuilder);
FWD(NodeAdapterFactory);
FWD(Port);

FWD(WidgetController);


class Designer;
class DesignBoard;
class DesignerScene;
class DesignerView;
class MinimapWidget;

class MovableGraphicsProxyWidget;
class ProfilingWidget;

class DragIO;
FWD(DragIOHandler);
}

#undef FWD

#endif // VIEW_FWD_H

