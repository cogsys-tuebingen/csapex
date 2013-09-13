#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

/// COMPONENT
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/model/boxed_object.h>
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_plugin/plugin_manager.hpp>
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <QLayout>
#include <QMenu>
#include <vector>
#include <QTreeWidget>

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
    static bool typeIsTemplate(const std::string& type);
    static std::string getTemplateName(const std::string& type);

    virtual void reload();
    ~BoxManager();

public:
    void register_box_type(BoxedObjectConstructor::Ptr provider);

    void startPlacingBox(QWidget *parent, const std::string& type, const QPoint &offset = QPoint(0,0));
    BoxPtr makeBox(const std::string& type, const std::string& uuid);
    BoxedObjectConstructor::Ptr getSelector(const std::string& type);

    void setContainer(QWidget* c);
    QWidget* container();

    void insertAvailableBoxedObjects(QMenu* menu);
    void insertAvailableBoxedObjects(QTreeWidget *tree);

    void stop();
    void reset();

public:
    boost::signals2::signal<void(const std::string&)> loaded;


protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);

    void rebuildMap();

    BoxPtr makeSingleBox(BoxedObjectConstructor::Ptr content, const std::string uuid, const std::string type);
    BoxPtr makeTemplateBox(const std::string uuid, const std::string type);

protected:
    std::vector<BoxedObjectConstructor::Ptr> available_elements_prototypes;

    QWidget* container_;

    std::map<Tag, std::vector<BoxedObjectConstructor::Ptr> > map;
    std::set<Tag> tags;

    PluginManager<BoxedObject>* manager_;

    bool dirty_;
};

}


namespace plugin_manager {
template <>
struct InstallConstructor<csapex::BoxedObject>
{
    template <class M, class L>
    static void installConstructor(M*, L* loader, const std::string& name, const std::string& description) {
        csapex::BoxedObjectConstructor::Ptr prototype(new csapex::BoxedObjectConstructor(name, description, boost::bind(&M::Loader::createInstance, loader, name)));
        csapex::BoxManager::instance().register_box_type(prototype);
    }
};
}


#endif // BOX_MANAGER_H
