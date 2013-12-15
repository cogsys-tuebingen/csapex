#ifndef BOX_DIALOG_H
#define BOX_DIALOG_H

/// SYSTEM
#include <QDialog>
#include <QLineEdit>
#include <QStyledItemDelegate>

class QAbstractItemModel;
class QListView;
class QStringListModel;
class QModelIndex;

namespace csapex
{
class HTMLDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    HTMLDelegate(int line_height);

public Q_SLOTS:
    void setKeyWords (const QString& words);

protected:
    void paint ( QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index ) const;

    QSize sizeHint ( const QStyleOptionViewItem & option, const QModelIndex & index ) const;

    int line_height;
    QStringList key_words;
};


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

private:
    QListView *list_view;

    int line_height;
};

class BoxDialog : public QDialog
{
    Q_OBJECT

public:
    BoxDialog(QWidget *parent = 0, Qt::WindowFlags f = 0);

    std::string getName();

private Q_SLOTS:
    void finish();

private:
    void makeUI();

private:
    CompleteLineEdit * name_edit_;
};

}

#endif // BOX_DIALOG_H
