#ifndef DATABASE_ITEM_MODEL_DECORATOR_H
#define DATABASE_ITEM_MODEL_DECORATOR_H

/// SYSTEM
#include <QAction>
#include <QFile>
#include <QTreeWidget>
#include <QShortcut>

/// FORWARD DECLARATION
class Database;
class MatchablePose;

/**
 * @brief The DatabaseItemModelDecorator is a Helper to put Database poses into a QListWidget
 */
class DatabaseItemModelDecorator : public QObject
{
    Q_OBJECT
public:
    static const std::string FILE_EXTENSION;

public:
    /**
     * @brief DatabaseItemModelDecorator
     * @param db Database to render
     * @param list list to put poses into
     */
    DatabaseItemModelDecorator(Database* db, QTreeWidget* tree);

    /**
     * @brief ~DatabaseItemModelDecorator
     */
    virtual ~DatabaseItemModelDecorator();

    void addComposite(int level, const std::string& label);
    void addLeaf(int level, MatchablePose* leaf);

private:
    QTreeWidgetItem* makeItem(int level, MatchablePose* pose);
    QTreeWidgetItem* makeItem(int level, const std::string& text);
    void cleanItems();

public Q_SLOTS:
    void save();
    void saveAs();
    void load();
    void loadCurrentFile();

    void rebuild();
    void rebuildRequest();
    bool deleteCurrentRow();

Q_SIGNALS:

private:
    QFile current_db_file;

    Database* db;

    QTreeWidget* tree;
    QTreeWidgetItem* current_target;

    QShortcut* delete_shortcut;

    QSize image_size;
    QTimer* repaint_timer;
};

#endif // DATABASE_ITEM_MODEL_DECORATOR_H
