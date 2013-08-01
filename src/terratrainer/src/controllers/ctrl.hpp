#ifndef CTRL_HPP
#define CTRL_HPP
#include <boost/shared_ptr.hpp>
class Controller
{
public:
    typedef boost::shared_ptr<Controller> Ptr;
protected:
    Controller()
    {
    }
};
#endif // CTRL_HPP

