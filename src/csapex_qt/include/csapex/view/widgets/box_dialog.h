#ifndef BOX_DIALOG_H
#define BOX_DIALOG_H

/// COMPONENT
#include <csapex/view/view_fwd.h>
#include <csapex/view/widgets/completed_line_edit.h>
#include <csapex_qt_export.h>

/// SYSTEM
#include <QDialog>
#include <QFuture>

class QStringListModel;
class QModelIndex;
class QProgressBar;
class QStandardItemModel;

namespace csapex
{
class NodeFactory;
class NodeFilterProxyModel;

class SnippetFactory;
typedef std::shared_ptr<SnippetFactory> SnippetFactoryPtr;


class CSAPEX_QT_EXPORT BoxDialog : public QDialog
{
    Q_OBJECT

public:
    BoxDialog(QString message, csapex::NodeFactory &node_factory,
              NodeAdapterFactory& adapter_factory,
              SnippetFactoryPtr snippet_factory,
              QWidget *parent = 0,
              Qt::WindowFlags f = 0);

    std::string getMIME();
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
    CompletedLineEdit * name_edit_;

    NodeFactory& node_factory_;
    NodeAdapterFactory& adapter_factory_;

    SnippetFactoryPtr snippet_factory_;

    QProgressBar* loading_;

    QString message_;

    NodeFilterProxyModel* filter;

    QFuture<bool> load_nodes;

    QStandardItemModel* model_;
};

}

#endif // BOX_DIALOG_H
