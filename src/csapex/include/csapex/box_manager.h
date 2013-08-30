#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

/// COMPONENT
#include <csapex/selector_proxy.h>
#include <csapex/boxed_object.h>
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_plugin/plugin_manager.hpp>
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <QLayout>
#include <QMenu>
#include <vector>
#include <QTreeWidget>


namespace plugin_manager {
template <>
struct InstallConstructor<csapex::BoxedObject>
{
    template <class M, class L>
    static void installConstructor(M*, L* loader, const std::string& name, const std::string& description) {
        csapex::SelectorProxy::Ptr dynamic(new csapex::SelectorProxyDynamic(name, description, boost::bind(&M::Loader::createInstance, loader, name)));
        csapex::SelectorProxy::registerProxy(dynamic);
    }
};
}

namespace csapex
{

class BoxManager : public QObject, public Singleton<BoxManager>
{
    Q_OBJECT

    friend class Singleton<BoxManager>;
    friend class DesignerIO;
    friend class GraphIO;

public:
    static std::string stripNamespace(const std::string& name);

    virtual void reload();
    ~BoxManager();

public:
    void register_box_type(SelectorProxy::ProxyConstructor provider);
    void register_box_type(SelectorProxy::Ptr provider);

    void startPlacingMetaBox(QWidget *parent, const QPoint &offset = QPoint(0,0));
    void startPlacingBox(QWidget *parent, const std::string& type, const QPoint &offset = QPoint(0,0));
    BoxPtr makeBox(QPoint pos, const std::string& type, const std::string& uuid = "");
    SelectorProxy::Ptr getSelector(const std::string& type);

    void setContainer(QWidget* c);
    QWidget* container();

    void insertAvailableBoxedObjects(QMenu* menu);
    void insertAvailableBoxedObjects(QTreeWidget *tree);

    std::string makeUUID(const std::string& name);
    void stop();

public:
    boost::signals2::signal<void(const std::string&)> loaded;


protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);

    std::vector<SelectorProxy::Ptr> available_elements_prototypes;

    std::map<std::string, int> uuids;

    QWidget* container_;

    std::map<Tag, std::vector<SelectorProxy::Ptr> > map;
    std::set<Tag> tags;

    PluginManager<BoxedObject>* manager_;
};

}

#endif // BOX_MANAGER_H
