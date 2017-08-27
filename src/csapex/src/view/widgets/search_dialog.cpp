/// HEADER
#include <csapex/view/widgets/search_dialog.h>

/// COMPONENT
#include <csapex/factory/node_factory.h>
#include <csapex/view/node/node_filter_proxy_model.h>
#include <csapex/view/utility/html_delegate.h>
#include <csapex/view/utility/node_list_generator.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/node_state.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/graph/vertex.h>


/// SYSTEM
#include <QLabel>
#include <QKeyEvent>
#include <QListView>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <QStandardItemModel>

using namespace csapex;


SearchDialog::SearchDialog(Graph *root, csapex::NodeFactory &node_factory, QString message, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f),
      root_(root),
      node_factory_(node_factory),
      message_(message)
{
    makeUI();
}

void SearchDialog::makeUI()
{
    setWindowIcon(QIcon(":/magnifier.png"));
    setWindowTitle("Search Node");
    setWindowFlags(Qt::WindowStaysOnTopHint);

    setFocusPolicy(Qt::StrongFocus);
    setModal(true);

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    QLabel *lbl = new QLabel(QString("<img src=\":/magnifier.png\"> ") + message_ + " (<em>Autocompleted</em>)");

    layout->addWidget(lbl);

    setupTextBox();
}

void SearchDialog::showEvent(QShowEvent * e)
{
    QDialog::showEvent(e);

}

void SearchDialog::setupTextBox()
{
    name_edit_ = new CompletedLineEdit;
    name_edit_->setFixedWidth(350);

    filter = new NodeFilterProxyModel;
    filter->setFilterCaseSensitivity(Qt::CaseInsensitive);

    QObject::connect(name_edit_, SIGNAL(textChanged(QString)), filter, SLOT(setFilterFixedString(const QString &)));
    QObject::connect(name_edit_, SIGNAL(textChanged(QString)), filter, SLOT(invalidate()));

    name_edit_->setModel(filter);

    layout()->addWidget(name_edit_);

    name_edit_->setFocus();

    model_ = listNodes();
    filter->setSourceModel(model_);

    QObject::connect(name_edit_, SIGNAL(editingFinished()), this, SLOT(finish()));
}

void SearchDialog::finish()
{
    if(getAUUID().empty()) {
        Q_EMIT reject();
    } else {
        Q_EMIT accept();
    }
}

AUUID SearchDialog::getAUUID()
{
    std::string auuid = name_edit_->text().toStdString();
    return AUUID(UUIDProvider::makeUUID_without_parent(auuid));
}

QAbstractItemModel* SearchDialog::listNodes()
{
    QStandardItemModel* model = new QStandardItemModel;

    addNodes(root_, model);

    model->sort(0);

    return model;
}

void SearchDialog::addNodes(Graph *graph, QStandardItemModel *model)
{
    for(const graph::VertexPtr& vtx : *graph) {
        NodeFacadePtr nf = vtx->getNodeFacade();

        AUUID auuid = nf->getAUUID();
        std::string type = nf->getType();
        std::string label = nf->getLabel();

        if(nf->isGraph()) {
            Graph* subgraph = graph->findSubgraph(nf->getUUID());
            addNodes(subgraph, model);
        }

        NodeConstructorPtr constr = node_factory_.getConstructor(type);

        QIcon icon = QIcon(QString::fromStdString(constr->getIcon()));
        QString label_s = QString::fromStdString(label);
        QString auuid_s = QString::fromStdString(auuid.getFullName());
        QString type_s = QString::fromStdString(type);

        QString auuid_html = auuid_s;
        auuid_html.replace(QString::fromStdString(UUID::namespace_separator),  " <b>&gt;</b> ");


        QStandardItem* item = new QStandardItem(icon, auuid_s);
        item->setData(auuid_html, Qt::UserRole);
        item->setData(type_s, Qt::UserRole + 1);
        item->setData(label_s, Qt::UserRole + 2);

        model->appendRow(item);
    }
}


/// MOC
#include "../../../include/csapex/view/widgets/moc_search_dialog.cpp"
