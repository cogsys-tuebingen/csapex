#ifndef FACTORY_FWD_H
#define FACTORY_FWD_H

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
FWD(NodeFactory);
FWD(SnippetFactory);
FWD(MessageFactory);
}

#undef FWD

#endif // FACTORY_FWD_H

