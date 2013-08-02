#ifndef CTRL_FACTORY_H
#define CTRL_FACTORY_H
/// COMPONENT
#include "ctrl_cmpcore_bridge.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>

/// DECLARATIONS
class TerraTrainerWindow;

/// CONTROLLERS
namespace Controller {
    typedef boost::shared_ptr<QObject> Ptr;
    enum ControllerID {ToolPanel, Menu, MapView, Bridge, Class, Preferences};
    typedef std::pair<ControllerID, Ptr> IDPtr;
    typedef std::map <ControllerID, Ptr> Map;

    template<class T, class U>
    boost::shared_ptr<T> to(boost::shared_ptr<U> const & r)
    {
       return boost::dynamic_pointer_cast<T>(r);
    }

}

class CtrlFactory
{
    friend class TerraTrainerWindow;
private:
    static  void produdeBridgeController (TerraTrainerWindow *mainWindow);
    static  void produceMapViewController(TerraTrainerWindow *mainWindow);
    static  void produceMenuController   (TerraTrainerWindow *mainWindow);
    static  void produceToolBarController(TerraTrainerWindow *mainWindow);
    static  void produceClassEdController(TerraTrainerWindow *mainWindow);
    static  void produceSettingController(TerraTrainerWindow *mainWindow);

private:
    CtrlFactory(){}
};

#endif // CTRL_FACTORY_H
