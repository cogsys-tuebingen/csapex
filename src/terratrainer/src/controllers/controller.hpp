#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <boost/shared_ptr.hpp>
#include <map>

namespace YAML {
class Emitter;
class Node;
}

class Controller {
public:
    enum ControllerID {ToolPanel, Menu, MapView, Bridge, Class, Preferences};

    typedef boost::shared_ptr<Controller> Ptr;
    typedef std::pair<ControllerID, Ptr> IDPtr;
    typedef std::map <ControllerID, Ptr> Map;

    virtual void read(const YAML::Node & document)
    {
        std::cout << "YAML: Called default reading method!" << std::endl;
    }
    virtual void write(YAML::Emitter &emitter)const
    {
        std::cout << "YAML: Called default writing method!" << std::endl;
    }


    template<class T, class U>
    static boost::shared_ptr<T> to(boost::shared_ptr<U> const & r)
    {
       return boost::dynamic_pointer_cast<T>(r);
    }

protected:
    Controller(){}

};



#endif // CONTROLLER_HPP
