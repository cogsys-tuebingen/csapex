#ifndef SEARCH_DIALOG_H
#define SEARCH_DIALOG_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/view_fwd.h>
#include <csapex/view/widgets/completed_line_edit.h>

/// PROJECT
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QDialog>
#include <QFuture>

class QListView;
class QStringListModel;
class QModelIndex;
class QProgressBar;
class QStandardItemModel;

namespace csapex
{
class NodeFilterProxyModel;
class Graph;
class NodeFactory;

class CSAPEX_QT_EXPORT SearchDialog : public QDialog
{
    Q_OBJECT

public:
    SearchDialog(Graph* root, csapex::NodeFactory &node_factory, QString message, QWidget *parent = 0, Qt::WindowFlags f = 0);

    AUUID getAUUID();

    void showEvent(QShowEvent*) override;

private Q_SLOTS:
    void setupTextBox();
    void finish();

Q_SIGNALS:
    void pluginsLoaded();

private:
    void makeUI();
    QAbstractItemModel* listNodes();

    void addNodes(Graph* graph, QStandardItemModel* model);

private:
    Graph* root_;
    csapex::NodeFactory &node_factory_;

    CompletedLineEdit * name_edit_;

    QString message_;

    NodeFilterProxyModel* filter;
    QAbstractItemModel* model_;
};

}

#endif // SEARCH_DIALOG_H
