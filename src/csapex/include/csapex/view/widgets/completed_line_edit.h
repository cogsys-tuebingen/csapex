#ifndef COMPLETED_LINE_EDIT_H
#define COMPLETED_LINE_EDIT_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QLineEdit>

class QListView;
class QAbstractItemModel;

namespace csapex
{

class CSAPEX_QT_EXPORT CompletedLineEdit : public QLineEdit
{
    Q_OBJECT

public:
    CompletedLineEdit(QWidget *parent = 0);

    std::string getMIME() const;
    std::string getName() const;

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

    std::string mime_;
};
}

#endif // COMPLETED_LINE_EDIT_H
