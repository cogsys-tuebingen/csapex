#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

/// COMPONENT
#include <csapex/selector_proxy.h>
#include <csapex/command.h>
#include <csapex/boxed_object.h>
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
    Box* makeBox(QWidget* parent, QPoint pos, const std::string& type, const std::string& uuid = "");

    Box* findBox(const std::string& uuid);
    Box* findConnectorOwner(const std::string& uuid);

    void setContainer(QWidget* c);
    QWidget* container();

    void fill(QLayout* layout);
    void fill(QMenu* menu);

    void execute(Command::Ptr command);

    bool isDirty();

    bool canUndo();
    bool canRedo();

    std::string makeUUID(const std::string& name);

    bool isBoxSelected(Box* box);
    int noSelectedBoxes();


    bool mouseMoveEventHandler(QMouseEvent * e);
    bool mousePressEventHandler(QMouseEvent * e);
    bool mouseReleaseEventHandler(QMouseEvent * e);

    bool keyPressEventHandler(QKeyEvent* e);
    bool keyReleaseEventHandler(QKeyEvent* e);

public Q_SLOTS:
    void undo();
    void redo();
    void selectBox(Box* box, bool add = false);
    void deselectBox(Box* box);
    void deselectBoxes();
    void toggleBoxSelection(Box* box);
    void boxMoved(Box* box, int dx, int dy);
    void setDirty();
    void resetDirtyPoint();
    void clearSavepoints();

Q_SIGNALS:
    void stateChanged();
    void dirtyChanged(bool);

private:
    void setDirty(bool dirty);

protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);

    std::vector<SelectorProxy::Ptr> available_elements_prototypes;

    std::deque<Command::Ptr> done;
    std::deque<Command::Ptr> undone;

    std::map<std::string, int> uuids;

    std::vector<Box*> selected_boxes;

    QWidget* container_;
    bool dirty_;
};

}

#endif // BOX_MANAGER_H
