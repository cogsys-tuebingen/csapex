#ifndef BOX_DIALOG_H
#define BOX_DIALOG_H

/// COMPONENT
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QDialog>
#include <QLineEdit>
#include <QFuture>

class QAbstractItemModel;
class QListView;
class QStringListModel;
class QModelIndex;
class QProgressBar;

namespace csapex
{
class NodeFactory;
class NodeFilterProxyModel;

class CompleteLineEdit : public QLineEdit
{
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
    BoxDialog(QString message, csapex::NodeFactory &node_factory, NodeAdapterFactory& adapter_factory, QWidget *parent = 0, Qt::WindowFlags f = 0);

    std::string getName();

    void showEvent(QShowEvent*) override;

private Q_SLOTS:
    void setupTextBox();
    void finish();

Q_SIGNALS:
    void pluginsLoaded();

private:
    void makeUI();

private:
    CompleteLineEdit * name_edit_;
    NodeFactory& node_factory_;
    NodeAdapterFactory& adapter_factory_;

    QProgressBar* loading_;

    QString message_;

    NodeFilterProxyModel* filter;

    QFuture<bool> load_nodes;

    QAbstractItemModel* model_;
};

}

#endif // BOX_DIALOG_H
