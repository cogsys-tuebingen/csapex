#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

/// COMPONENT
#include <csapex/selector_proxy.h>
#include <csapex/boxed_object.h>

/// PROJECT
#include <utils_plugin/plugin_manager.hpp>

/// SYSTEM
#include <QLayout>
#include <QMenu>
#include <vector>



namespace plugin_manager {
template <>
struct InstallConstructor<csapex::BoxedObject>
{
    template <class M, class L>
    static void installConstructor(M* instance, L* loader, const std::string& name) {
        csapex::SelectorProxy::Ptr dynamic(new csapex::SelectorProxyDynamic(name, boost::bind(&M::Loader::createUnmanagedInstance, loader, name)));
        csapex::SelectorProxy::registerProxy(dynamic);
    }
};
}

namespace csapex
{

class BoxManager : public QObject, public PluginManager<BoxedObject>
{
    Q_OBJECT

    friend class DesignerIO;
    friend class GraphIO;

public:
    static BoxManager& instance() {
        static BoxManager inst;
        return inst;
    }

    static std::string stripNamespace(const std::string& name);

public:
    void register_box_type(SelectorProxy::ProxyConstructor provider);
    void register_box_type(SelectorProxy::Ptr provider);

    void startPlacingBox(const std::string& type, const QPoint &offset = QPoint(0,0));
    Box* makeBox(QPoint pos, const std::string& type, const std::string& uuid = "");

    void setContainer(QWidget* c);
    QWidget* container();

    void insertAvailableBoxedObjects(QLayout* layout);
    void insertAvailableBoxedObjects(QMenu* menu);

    std::string makeUUID(const std::string& name);

protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);

    std::vector<SelectorProxy::Ptr> available_elements_prototypes;

    std::map<std::string, int> uuids;

    QWidget* container_;
};

}

#endif // BOX_MANAGER_H
