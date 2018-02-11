/// HEADER
#include <csapex/view/utility/snippet_list_generator.h>

/// COMPONENT
#include <csapex/factory/snippet_factory.h>
#include <csapex/serialization/snippet.h>
#include <csapex/model/tag.h>
#include <csapex/csapex_mime.h>

/// SYSTEM
#include <QTreeWidget>
#include <QStandardItemModel>

using namespace csapex;

SnippetListGenerator::SnippetListGenerator(SnippetFactory &snippet_factory)
    : snippet_factory_(snippet_factory)
{

}

void SnippetListGenerator::insertAvailableSnippets(QMenu* menu)
{
    auto tags = snippet_factory_.getTagMap();

    for(const auto& pair : tags) {
        const std::string& tag = pair.first;
        const std::vector<SnippetPtr>& constructors = pair.second;

        QMenu* submenu = new QMenu(QString::fromStdString(tag));
        menu->addMenu(submenu);

        for(const SnippetPtr& snippet : constructors) {
            QIcon icon = QIcon(":/snippet.png");


            QAction* action = new QAction(snippet->getName().c_str(), submenu);
            action->setData(QString(snippet->getName().c_str()));
            if(!icon.isNull()) {
                action->setIcon(icon);
                action->setIconVisibleInMenu(true);
            }
            action->setToolTip(snippet->getDescription().c_str());
            action->setData(QVariant::fromValue(QPair<QString, QString>(QString::fromStdString(csapex::mime::snippet), QString::fromStdString(snippet->getName()))));
            submenu->addAction(action);
        }
    }

    menu->menuAction()->setIconVisibleInMenu(true);

}


void SnippetListGenerator::insertAvailableSnippets(QTreeWidget* tree)
{
    auto tags = snippet_factory_.getTagMap();

    tree->setDragEnabled(true);

    for(const auto& pair : tags) {
        const std::string& tag = pair.first;
        const std::vector<SnippetPtr>& constructors = pair.second;

        QTreeWidgetItem* submenu = new QTreeWidgetItem;
        submenu->setText(0, QString::fromStdString(tag));
        tree->addTopLevelItem(submenu);

        for(const SnippetPtr& snippet : constructors) {
            QIcon icon = QIcon(":/snippet.png");
            std::string name = snippet->getName();

            QTreeWidgetItem* child = new QTreeWidgetItem;
            child->setToolTip(0, (snippet->getName() + ": " + snippet->getDescription()).c_str());
            child->setIcon(0, icon);
            child->setText(0, name.c_str());
            child->setData(0, Qt::UserRole, QString::fromStdString(csapex::mime::snippet));
            child->setData(0, Qt::UserRole + 1, snippet->getName().c_str());

            submenu->addChild(child);
        }
    }
}

void SnippetListGenerator::listAvailableSnippets(QStandardItemModel* model)
{
    for(const auto& pair: snippet_factory_.getSnippets()) {
        const SnippetPtr& snippet = pair.second;

        QString name = QString::fromStdString(snippet->getName());
        QString descr(snippet->getDescription().c_str());
        QString type(snippet->getName().c_str());

        QStringList tags;
        for(const TagConstPtr& tag : snippet->getTags()) {
            tags << tag->getName().c_str();
        }

        QStringList properties;
        properties.push_back("snippet");

        QStandardItem* item = new QStandardItem(QIcon(":/snippet"), type);
        item->setData(type, Qt::UserRole);
        item->setData(descr, Qt::UserRole + 1);
        item->setData(name, Qt::UserRole + 2);
        item->setData(tags, Qt::UserRole + 3);
        item->setData(properties, Qt::UserRole + 4);
        item->setData(QString::fromStdString(csapex::mime::snippet), Qt::UserRole + 5);

        model->appendRow(item);
    }
}
