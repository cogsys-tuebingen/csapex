#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

/// COMPONENT
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <QLayout>
#include <QMenu>
#include <vector>
#include <QTreeWidget>
#include <boost/signals2.hpp>
#include <set>

namespace csapex
{

class BoxManager : public Singleton<BoxManager>
{
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
    void register_box_type(BoxedObjectConstructor::Ptr provider, bool suppress_signals = false);

    void startPlacingBox(QWidget *parent, const std::string& type, const QPoint &offset = QPoint(0,0));
    NodePtr makeNode(const std::string& type, const std::string& uuid);
    BoxedObjectConstructor::Ptr getSelector(const std::string& type);

    void setContainer(QWidget* c);
    QWidget* container();

    void insertAvailableBoxedObjects(QMenu* menu);
    void insertAvailableBoxedObjects(QTreeWidget *tree);

    void stop();

public:
    boost::signals2::signal<void(const std::string&)> loaded;
    boost::signals2::signal<void()> new_box_type;


protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);

    void ensureLoaded();
    void rebuildPrototypes();
    void rebuildMap();

    NodePtr makeSingleNode(BoxedObjectConstructor::Ptr content, const std::string& uuid);
    NodePtr makeTemplateNode(const std::string& uuid, const std::string& type);

protected:
    std::vector<BoxedObjectConstructor::Ptr> available_elements_prototypes;

    QWidget* container_;

    std::map<Tag, std::vector<BoxedObjectConstructor::Ptr> > map;
    std::set<Tag> tags;

    PluginManager<BoxedObject>* manager_;

    bool dirty_;
};

}

#endif // BOX_MANAGER_H
