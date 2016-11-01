/// HEADER
#include <csapex/view/utility/node_list_generator.h>

/// COMPONENT
#include <csapex/model/node_constructor.h>
#include <csapex/factory/node_factory.h>
#include <csapex/view/node/box.h>
#include <csapex/model/tag.h>
#include <csapex/csapex_mime.h>

/// SYSTEM
#include <QTreeWidget>
#include <QStandardItemModel>

using namespace csapex;

NodeListGenerator::NodeListGenerator(NodeFactory &node_factory, NodeAdapterFactory &adapter_factory)
    : node_factory_(node_factory), adapter_factory_(adapter_factory)
{

}

void NodeListGenerator::insertAvailableNodeTypes(QMenu* menu)
{
    auto tags = node_factory_.getTagMap();

    for(const auto& pair : tags) {
        const std::string& tag = pair.first;
        const std::vector<NodeConstructor::Ptr>& constructors = pair.second;

        QMenu* submenu = new QMenu(QString::fromStdString(tag));
        menu->addMenu(submenu);

        for(const NodeConstructor::Ptr& proxy : constructors) {
            QIcon icon = QIcon(QString::fromStdString(proxy->getIcon()));
            QAction* action = new QAction(UUID::stripNamespace(proxy->getType()).c_str(), submenu);
            action->setData(QString(proxy->getType().c_str()));
            if(!icon.isNull()) {
                action->setIcon(icon);
                action->setIconVisibleInMenu(true);
            }
            action->setToolTip(proxy->getDescription().c_str());
            action->setData(QVariant::fromValue(QPair<QString, QString>(QString::fromStdString(csapex::mime::node), QString::fromStdString(proxy->getType()))));
            submenu->addAction(action);
        }
    }

    menu->menuAction()->setIconVisibleInMenu(true);

}


void NodeListGenerator::insertAvailableNodeTypes(QTreeWidget* tree)
{
    auto tags = node_factory_.getTagMap();

    tree->setDragEnabled(true);

    for(const auto& pair : tags) {
        const std::string& tag = pair.first;
        const std::vector<NodeConstructor::Ptr>& constructors = pair.second;

        QTreeWidgetItem* submenu = new QTreeWidgetItem;
        submenu->setText(0, QString::fromStdString(tag));
        tree->addTopLevelItem(submenu);

        for(const NodeConstructor::Ptr& proxy : constructors) {
            QIcon icon = QIcon(QString::fromStdString(proxy->getIcon()));
            std::string name = UUID::stripNamespace(proxy->getType());

            QTreeWidgetItem* child = new QTreeWidgetItem;
            child->setToolTip(0, (proxy->getType() + ": " + proxy->getDescription()).c_str());
            child->setIcon(0, icon);
            child->setText(0, name.c_str());
            child->setData(0, Qt::UserRole, QString::fromStdString(csapex::mime::node));
            child->setData(0, Qt::UserRole + 1, proxy->getType().c_str());

            submenu->addChild(child);
        }
    }
}

void NodeListGenerator::listAvailableNodeTypes(QStandardItemModel* model)
{
    for(const NodeConstructor::Ptr& proxy : node_factory_.getConstructors()) {
        QString name = QString::fromStdString(UUID::stripNamespace(proxy->getType()));
        QString descr(proxy->getDescription().c_str());
        QString type(proxy->getType().c_str());

        QStringList tags;
        for(const Tag::Ptr& tag : proxy->getTags()) {
            tags << tag->getName().c_str();
        }

        QStringList properties;
        for(const std::string& s : proxy->getProperties()) {
            properties.append(QString::fromStdString(s));
        }

        if(adapter_factory_.hasAdapter(proxy->getType())) {
            properties.push_back("QT");
        }

        QStandardItem* item = new QStandardItem(QIcon(QString::fromStdString(proxy->getIcon())), type);
        item->setData(type, Qt::UserRole);
        item->setData(descr, Qt::UserRole + 1);
        item->setData(name, Qt::UserRole + 2);
        item->setData(tags, Qt::UserRole + 3);
        item->setData(properties, Qt::UserRole + 4);
        item->setData(QString::fromStdString(csapex::mime::node), Qt::UserRole + 5);

        model->appendRow(item);
    }
}
