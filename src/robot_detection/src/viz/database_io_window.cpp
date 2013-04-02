/// HEADER
#include "database_io_window.h"

/// COMPONENT
#include "database_item_model_decorator.h"

DatabaseIOWindow::DatabaseIOWindow(QWidget* parent)
    : QMainWindow(parent), db_model(NULL), save_action(NULL), save_as_action(NULL), load_action(NULL)
{
}

DatabaseIOWindow::~DatabaseIOWindow()
{
}

void DatabaseIOWindow::modelReplaced()
{
    save_action = findChild<QAction*>("actionSave");
    save_as_action = findChild<QAction*>("actionSaveAs");
    load_action = findChild<QAction*>("actionLoad");

    QObject::connect(save_action, SIGNAL(triggered()), db_model, SLOT(save()));
    QObject::connect(save_as_action, SIGNAL(triggered()), db_model, SLOT(saveAs()));
    QObject::connect(load_action, SIGNAL(triggered()), db_model, SLOT(load()));
}
