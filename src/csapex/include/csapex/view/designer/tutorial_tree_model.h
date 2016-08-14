#ifndef TUTORIAL_TREE_MODEL_H
#define TUTORIAL_TREE_MODEL_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QStandardItemModel>
#include <QFile>

class QTreeWidget;
class QTreeWidgetItem;

namespace csapex
{

class Settings;


class CSAPEX_QT_EXPORT TutorialTreeModel
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
