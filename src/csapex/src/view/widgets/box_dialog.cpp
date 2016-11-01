/// HEADER
#include <csapex/view/widgets/box_dialog.h>

/// COMPONENT
#include <csapex/factory/node_factory.h>
#include <csapex/view/node/node_filter_proxy_model.h>
#include <csapex/view/utility/html_delegate.h>
#include <csapex/view/utility/node_list_generator.h>
#include <csapex/view/utility/snippet_list_generator.h>

/// SYSTEM
#include <QLabel>
#include <QKeyEvent>
#include <QListView>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <QProgressBar>
#include <QTimer>
#include <QStandardItemModel>

using namespace csapex;



BoxDialog::BoxDialog(QString message, NodeFactory& node_factory, NodeAdapterFactory &adapter_factory, SnippetFactory& snippet_factory, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f), node_factory_(node_factory), adapter_factory_(adapter_factory), snippet_factory_(snippet_factory),
      message_(message)
{
    makeUI();
}

void BoxDialog::makeUI()
{
    setWindowIcon(QIcon(":/add_node.png"));
    setWindowTitle("Create Node");
    setWindowFlags(Qt::WindowStaysOnTopHint);

    setFocusPolicy(Qt::StrongFocus);
    setModal(true);

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    QLabel *lbl = new QLabel(QString("<img src=\":/add_node.png\"> ") + message_ + " (<em>Autocompleted</em>)");

    layout->addWidget(lbl);

    loading_ = new QProgressBar;
    loading_->setTextVisible(true);
    loading_->setValue(-1);
    loading_->setRange(0,0);
    loading_->setFormat("Loading plugins..");

    layout->addWidget(loading_);

    QObject::connect(this, &BoxDialog::pluginsLoaded, this, &BoxDialog::setupTextBox);
}

void BoxDialog::showEvent(QShowEvent * e)
{
    QDialog::showEvent(e);

    if(!load_nodes.isRunning()) {
        load_nodes = QtConcurrent::run([this](){
            model_ = new QStandardItemModel;

            NodeListGenerator node_generator(node_factory_, adapter_factory_);
            node_generator.listAvailableNodeTypes(model_);

            SnippetListGenerator snippet_generator(snippet_factory_);
            snippet_generator.listAvailableSnippets(model_);

            model_->sort(0);

            Q_EMIT pluginsLoaded();

            return true;
        });
    }
}

void BoxDialog::setupTextBox()
{
    name_edit_ = new CompletedLineEdit;
    name_edit_->setFixedWidth(350);

    filter = new NodeFilterProxyModel;
    filter->setFilterCaseSensitivity(Qt::CaseInsensitive);

    QObject::connect(name_edit_, &CompletedLineEdit::textChanged, filter, &NodeFilterProxyModel::setFilterFixedString);;
    QObject::connect(name_edit_, &CompletedLineEdit::textChanged, filter, &NodeFilterProxyModel::invalidate);

    name_edit_->setModel(filter);

    delete loading_;
    layout()->addWidget(name_edit_);


    filter->setSourceModel(model_);

    QObject::connect(name_edit_, SIGNAL(editingFinished()), this, SLOT(finish()));

    QTimer::singleShot(0, name_edit_, SLOT(setFocus()));
}

void BoxDialog::finish()
{
    if(load_nodes.isRunning()) {
        load_nodes.waitForFinished();
    }
    if(getName().empty()) {
        Q_EMIT reject();
    } else {
        Q_EMIT accept();
    }
}

std::string BoxDialog::getMIME()
{
    return name_edit_->getMIME();
}

std::string BoxDialog::getName()
{
    return name_edit_->getName();
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_box_dialog.cpp"
