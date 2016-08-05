#ifndef COMPLETED_LINE_EDIT_H
#define COMPLETED_LINE_EDIT_H

/// SYSTEM
#include <QLineEdit>

class QListView;
class QAbstractItemModel;

namespace csapex
{

class CompletedLineEdit : public QLineEdit
{
    Q_OBJECT

public:
    CompletedLineEdit(QWidget *parent = 0);

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
}

#endif // COMPLETED_LINE_EDIT_H
