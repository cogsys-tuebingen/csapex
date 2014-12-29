#ifndef PARAM_FWD_H
#define PARAM_FWD_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

#define FWD(name) \
    class name;\
    typedef boost::shared_ptr<name> name##Ptr;\
    typedef boost::shared_ptr<const name> name##ConstPtr;\
    static const name##Ptr name##NullPtr

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

FWD(OutputProgressParameter);

}

#undef FWD

#endif // PARAM_FWD_H
