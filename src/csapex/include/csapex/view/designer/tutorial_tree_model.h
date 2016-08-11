#ifndef TUTORIAL_TREE_MODEL_H
#define TUTORIAL_TREE_MODEL_H

/// SYSTEM
#include <QStandardItemModel>
#include <QFile>

class QTreeWidget;
class QTreeWidgetItem;

namespace csapex
{

class Settings;


class TutorialTreeModel
{
public:
    struct ReadMe
    {
        QString title;
        QString description;
    };

public:
    TutorialTreeModel(Settings &settings);
    ~TutorialTreeModel();

    void fill(QTreeWidget* tree);

private:
    template <typename Path>
    void importDirectory(QTreeWidgetItem* parent, const Path& path);

    ReadMe parseReadMe(QFile& file);

private:
    Settings &settings_;
    QTreeWidget *tree_;
};

}

#endif // TUTORIAL_TREE_MODEL_H
