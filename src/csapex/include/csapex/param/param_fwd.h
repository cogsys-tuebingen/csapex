#ifndef PARAM_FWD_H
#define PARAM_FWD_H

/// SYSTEM
#include <memory>

#define FWD(name) \
    class name;\
    typedef std::shared_ptr<name> name##Ptr;\
    typedef std::unique_ptr<name> name##UniquePtr;\
    typedef std::weak_ptr<name> name##WeakPtr;\
    typedef std::shared_ptr<const name> name##ConstPtr;

namespace csapex {
namespace param
{

FWD(Parameter);
FWD(BitSetParameter);
FWD(SetParameter);
FWD(PathParameter);
FWD(RangeParameter);
FWD(TriggerParameter);
FWD(ValueParameter);
FWD(IntervalParameter);
FWD(ColorParameter);
FWD(AngleParameter);
FWD(StringListParameter);

FWD(OutputProgressParameter);
FWD(OutputTextParameter);

}
}

#undef FWD

#endif // PARAM_FWD_H
