/// HEADER
#include <csapex/box_manager.h>

/// COMPONENT
#include <csapex/selector_proxy.h>
#include <csapex/command_meta.h>
#include <csapex/command_delete_box.h>
#include <csapex/box.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QApplication>
#include <stack>

using namespace csapex;

BoxManager::BoxManager()
    : PluginManager<csapex::BoxedObject>("csapex::BoxedObject"), dirty_(false)
{
}

void BoxManager::fill(QLayout* layout)
{
    if(!pluginsLoaded()) {
        reload();
    }

    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        layout->addWidget(p->clone());
    }
}

namespace {
bool compare (SelectorProxy::Ptr a, SelectorProxy::Ptr b) {
    const std::string& as = BoxManager::stripNamespace(a->getType());
    const std::string& bs = BoxManager::stripNamespace(b->getType());
    return as.compare(bs) < 0;
}
}

void BoxManager::fill(QMenu* menu)
{
    if(!pluginsLoaded()) {
        reload();
    }

    std::map<std::string, std::vector<SelectorProxy::Ptr> > map;
    std::set<std::string> categories;

    std::string no_cat = "General";

    categories.insert(no_cat);

    foreach(SelectorProxy::Ptr p, available_elements_prototypes) {
        std::string cat = p->getCategory();
        if(cat.empty()) {
            cat = no_cat;
        }
        map[cat].push_back(p);
        categories.insert(cat);
    }

    foreach(const std::string& cat, categories) {
        QMenu* submenu = new QMenu(cat.c_str());
        menu->addMenu(submenu);

        std::sort(map[cat].begin(), map[cat].end(), compare);

        foreach(const SelectorProxy::Ptr& proxy, map[cat]) {
            QIcon icon = proxy->getIcon();
            QAction* action = new QAction(stripNamespace(proxy->getType()).c_str(), submenu);
            action->setData(QString(proxy->getType().c_str()));
            if(!icon.isNull()) {
                action->setIcon(icon);
                action->setIconVisibleInMenu(true);
            }
            submenu->addAction(action);
        }
    }

    menu->menuAction()->setIconVisibleInMenu(true);

}

void BoxManager::register_box_type(SelectorProxy::Ptr provider)
{
    available_elements_prototypes.push_back(provider);
}

void BoxManager::startPlacingBox(const std::string &type, const QPoint& offset)
{
    foreach(SelectorProxy::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            p->startObjectPositioning(offset);
            return;
        }
    }
}

std::string BoxManager::stripNamespace(const std::string &name)
{
    size_t from = name.find_first_of("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}

Box* BoxManager::makeBox(QWidget* parent, QPoint pos, const std::string& target_type, const std::string& uuid)
{
    std::string type = target_type;
    if(type.find_first_of(" ") != type.npos) {
        std::cout << "warning: type '" << type << "' contains spaces, stripping them!" << std::endl;
        while(type.find(" ") != type.npos) {
            type.replace(type.find(" "), 1, "");
        }
    }


    std::string uuid_ = uuid;
    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        if(p->getType() == type) {
            if(uuid_.empty()) {
                uuid_ = makeUUID(type);
            }
            return p->spawnObject(parent, pos, type, uuid);
        }
    }

    std::cout << "warning: cannot make box, type '" << type << "' is unknown, trying different namespace" << std::endl;

    std::string type_wo_ns = stripNamespace(type);

    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        std::string p_type_wo_ns = stripNamespace(p->getType());

        if(p_type_wo_ns == type_wo_ns) {
            if(uuid_.empty()) {
                uuid_ = makeUUID(type);
            }
            std::cout << "found a match: '" << type << " == " << p->getType() << std::endl;
            return p->spawnObject(parent, pos, p->getType(), uuid);
        }
    }

    std::cerr << "error: cannot make box, type '" << type << "' is unknown\navailable:\n";
    BOOST_FOREACH(SelectorProxy::Ptr p, available_elements_prototypes) {
        std::cerr << p->getType() << '\n';
    }
    std::cerr << std::endl;
    return NULL;
}

Box* BoxManager::findBox(const std::string &uuid)
{
    BOOST_FOREACH(Box* b, container_->findChildren<Box*>()) {
        if(b->UUID() == uuid) {
            return b;
        }
    }

    return NULL;
}

Box* BoxManager::findConnectorOwner(const std::string &uuid)
{
    BOOST_FOREACH(Box* b, container_->findChildren<Box*>()) {
        if(b->getInput(uuid)) {
            return b;
        }
        if(b->getOutput(uuid)) {
            return b;
        }
    }

    return NULL;
}

void BoxManager::setContainer(QWidget *c)
{
    container_ = c;
}

QWidget* BoxManager::container()
{
    return container_;
}

void BoxManager::execute(Command::Ptr command)
{
    if(!isDirty()) {
        command->setAfterSavepoint(true);
    }

    bool change = command->execute();
    done.push_back(command);

    while(!undone.empty()) {
        undone.pop_back();
    }

    if(change) {
        setDirty();
        Q_EMIT stateChanged();
    }
}

std::string BoxManager::makeUUID(const std::string& name)
{
    int& last_id = uuids[name];
    ++last_id;

    std::stringstream ss;
    ss << name << "_" << last_id;

    return ss.str();
}

bool BoxManager::isDirty()
{
    return dirty_;
}

void BoxManager::resetDirtyPoint()
{
    setDirty(false);

    clearSavepoints();

    if(!done.empty()) {
        done.back()->setBeforeSavepoint(true);
    }
    if(!undone.empty()) {
        undone.back()->setAfterSavepoint(true);
    }
}

void BoxManager::clearSavepoints()
{
    foreach(Command::Ptr cmd, done) {
        cmd->setAfterSavepoint(false);
        cmd->setBeforeSavepoint(false);
    }
    foreach(Command::Ptr cmd, undone) {
        cmd->setAfterSavepoint(false);
        cmd->setBeforeSavepoint(false);
    }
}

void BoxManager::setDirty()
{
    setDirty(true);
}

void BoxManager::setDirty(bool dirty)
{
    bool change = dirty_ != dirty;

    dirty_ = dirty;

    if(change) {
        Q_EMIT dirtyChanged(dirty_);
    }
}


bool BoxManager::canUndo()
{
    return !done.empty();
}

bool BoxManager::canRedo()
{
    return !undone.empty();
}

void BoxManager::undo()
{
    if(!canUndo()) {
        return;
    }

    Command::Ptr last = done.back();
    done.pop_back();

    bool ret = last->undo();
    assert(ret);

    setDirty(!last->isAfterSavepoint());

    undone.push_back(last);

    Q_EMIT stateChanged();
}

void BoxManager::redo()
{
    if(!canRedo()) {
        return;
    }

    Command::Ptr last = undone.back();
    undone.pop_back();

    last->redo();

    done.push_back(last);

    setDirty(!last->isBeforeSavepoint());

    Q_EMIT stateChanged();
}

bool BoxManager::isBoxSelected(Box *box)
{
    return std::find(selected_boxes.begin(), selected_boxes.end(), box) != selected_boxes.end();
}

int BoxManager::noSelectedBoxes()
{
    return selected_boxes.size();
}

void BoxManager::selectBox(Box *box, bool add)
{
    assert(!isBoxSelected(box));

    if(!add) {
        deselectBoxes();
        assert(selected_boxes.empty());
    }

    selected_boxes.push_back(box);

    box->selectEvent();
}


void BoxManager::deselectBox(Box *box)
{
    assert(isBoxSelected(box));

    selected_boxes.erase(std::find(selected_boxes.begin(), selected_boxes.end(), box));

    box->deselectEvent();
}
void BoxManager::deselectBoxes()
{
    foreach(Box* b, selected_boxes) {
        deselectBox(b);
        b->deselectEvent();
    }
}


void BoxManager::toggleBoxSelection(Box *box)
{
    bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();

    if(shift) {
        if(isBoxSelected(box)) {
            deselectBox(box);
        } else {
            selectBox(box, true);
        }
    } else {
        if(isBoxSelected(box)) {
            deselectBoxes();
            if(noSelectedBoxes() != 1) {
                selectBox(box);
            }
        } else {
            selectBox(box);
        }
    }
}

void BoxManager::boxMoved(Box *box, int dx, int dy)
{
    if(isBoxSelected(box) && box->hasFocus()) {
        foreach(Box* b, selected_boxes) {
            if(b != box) {
                b->move(b->x() + dx, b->y() + dy);
            }
        }
    }
}

bool BoxManager::mouseMoveEventHandler(QMouseEvent *e)
{
    return true;
}

bool BoxManager::keyPressEventHandler(QKeyEvent* e)
{
    return true;
}

bool BoxManager::keyReleaseEventHandler(QKeyEvent* e)
{
    if(e->key() == Qt::Key_Delete || e->key() == Qt::Key_Backspace) {
        if(selected_boxes.size() == 0) {
            return true;
        }

        command::Meta::Ptr meta(new command::Meta);

        foreach(Box* b, selected_boxes) {
            meta->add(Command::Ptr(new command::DeleteBox(b)));
        }

        deselectBoxes();

        execute(meta);

        return false;
    }

    return true;
}

bool BoxManager::mousePressEventHandler(QMouseEvent *e)
{
    return true;
}

bool BoxManager::mouseReleaseEventHandler(QMouseEvent *e)
{
    deselectBoxes();

    return true;
}
