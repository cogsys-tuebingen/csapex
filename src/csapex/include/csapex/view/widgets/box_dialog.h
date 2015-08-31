#ifndef BOX_DIALOG_H
#define BOX_DIALOG_H

/// COMPONENT
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QDialog>
#include <QLineEdit>

class QAbstractItemModel;
class QListView;
class QStringListModel;
class QModelIndex;

namespace csapex
{
class CompleteLineEdit : public QLineEdit {

    Q_OBJECT

public:
    CompleteLineEdit(QWidget *parent = 0);

public Q_SLOTS:
    void update();
    void setModel(QAbstractItemModel *completer);
    void completeText(const QModelIndex &index);

protected:
    virtual void keyPressEvent(QKeyEvent *e);
    virtual void focusOutEvent(QFocusEvent *e);
    virtual void focusInEvent(QFocusEvent* e);

private:
    QListView *list_view;

    bool was_hidden;
    int line_height;
};

class BoxDialog : public QDialog
{
    Q_OBJECT

public:
    BoxDialog(WidgetController *widget_controller, QWidget *parent = 0, Qt::WindowFlags f = 0);

    std::string getName();

private Q_SLOTS:
    void finish();

private:
    void makeUI();

private:
    CompleteLineEdit * name_edit_;
    WidgetController* widget_controller_;
};

}

#endif // BOX_DIALOG_H
