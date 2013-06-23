#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

/// COMPONENT
#include "selector_proxy.h"
#include "command.h"

/// PROJECT
#include <utils/plugin_manager.hpp>

/// SYSTEM
#include <QLayout>
#include <vector>

namespace vision_evaluator
{

class BoxManager : public QObject
{
    Q_OBJECT

    friend class DesignerIO;

public:
    static BoxManager& instance() {
        static BoxManager inst;
        return inst;
    }

public:
    void register_box_type(SelectorProxy::ProxyConstructor provider);
    void register_box_type(SelectorProxy::Ptr provider);

    Box* makeBox(QWidget* parent, QPoint pos, const std::string& type, const std::string& uuid = "");
    Box* findBox(const std::string& uuid);

    Box* findConnectorOwner(const std::string& uuid);

    void setContainer(QWidget* c);
    QWidget* container();

    void fill(QLayout* layout);

    void execute(Command::Ptr command);

    bool isDirty();
    void setDirty(bool dirty);

    bool canUndo();
    bool canRedo();

    std::string makeUUID(const std::string& name);

public Q_SLOTS:
    void undo();
    void redo();

Q_SIGNALS:
    void stateChanged();
    void dirtyChanged(bool);

protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);

    PluginManager<SelectorProxy, SelectorProxy::ProxyConstructor> available_elements;
    std::vector<SelectorProxy::Ptr> available_elements_prototypes;

    std::stack<Command::Ptr> done;
    std::stack<Command::Ptr> undone;

    std::map<std::string, int> uuids;

    QWidget* container_;
    bool dirty_;
};

}

#endif // BOX_MANAGER_H
