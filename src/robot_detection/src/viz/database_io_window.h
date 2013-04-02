#ifndef DATABASE_IO_WINDOW_H
#define DATABASE_IO_WINDOW_H

/// SYSTEM
#include <QMainWindow>

class DatabaseItemModelDecorator;

/**
 * @brief The DatabaseIOWindow class visualizes a Database
 */
class DatabaseIOWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief DatabaseIOWindow
     * @param parent
     */
    DatabaseIOWindow(QWidget* parent = 0);

    /**
     * @brief ~DatabaseIOWindow
     */
    virtual ~DatabaseIOWindow();

protected:
    void modelReplaced();

protected:
    DatabaseItemModelDecorator* db_model;

    QAction* save_action;
    QAction* save_as_action;
    QAction* load_action;
};

#endif // DATABASE_IO_WINDOW_H
