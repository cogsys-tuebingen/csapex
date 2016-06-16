/// HEADER
#include <csapex/view/widgets/box_dialog.h>

/// COMPONENT
#include <csapex/factory/node_factory.h>
#include <csapex/view/node/node_filter_proxy_model.h>
#include <csapex/view/utility/html_delegate.h>
#include <csapex/view/utility/node_list_generator.h>

/// SYSTEM
#include <QLabel>
#include <QKeyEvent>
#include <QListView>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <QProgressBar>

using namespace csapex;


CompleteLineEdit::CompleteLineEdit(QWidget *parent)
    : QLineEdit(parent), was_hidden(false)
{
    line_height = 20;

    list_view = new QListView(this);
    list_view->setWindowFlags(Qt::ToolTip);

    HTMLBoxDelegate* delegate = new HTMLBoxDelegate(line_height);
    list_view->setItemDelegate(delegate);

    connect(list_view, SIGNAL(clicked(const QModelIndex&)), this, SLOT(completeText(const QModelIndex&)));
    connect(this, SIGNAL(textChanged(QString)), delegate, SLOT(setKeyWords(const QString&)));
}


void CompleteLineEdit::focusOutEvent(QFocusEvent */*e*/) {
    list_view->hide();
    was_hidden = true;
}

void CompleteLineEdit::focusInEvent(QFocusEvent */*e*/) {
    // only show the view, if it has been visible before and then has been hidden!
    if(was_hidden) {
        list_view->show();
        update();
    }
}

void CompleteLineEdit::keyPressEvent(QKeyEvent *e) {
    if (!list_view->isHidden()) {
        int key = e->key();
        int count = list_view->model()->rowCount();
        QModelIndex currentIndex = list_view->currentIndex();

        if (Qt::Key_Down == key) {
            int row = currentIndex.row() + 1;
            if (row >= count) {
                row = 0;
            }

            QModelIndex index = list_view->model()->index(row, 0);
            list_view->setCurrentIndex(index);

        } else if (Qt::Key_Up == key) {
            int row = currentIndex.row() - 1;
            if (row < 0) {
                row = count - 1;
            }

            QModelIndex index = list_view->model()->index(row, 0);
            list_view->setCurrentIndex(index);

        } else if (Qt::Key_Escape == key) {
            list_view->hide();
            QLineEdit::keyPressEvent(e);

        } else if (Qt::Key_Enter == key || Qt::Key_Return == key) {
            if (currentIndex.isValid()) {
                completeText(list_view->currentIndex());
            }

            list_view->hide();

        } else {
            QLineEdit::keyPressEvent(e);
        }

    } else {
        QLineEdit::keyPressEvent(e);
    }
}


void CompleteLineEdit::setModel(QAbstractItemModel *model) {
    list_view->setModel(model);

    QObject::connect(model, SIGNAL(dataChanged(QModelIndex,QModelIndex)), this, SLOT(update()));
    QObject::connect(model, SIGNAL(layoutChanged()), this, SLOT(update()));
}

void CompleteLineEdit::update() {
    list_view->setMinimumWidth(width());
    list_view->setMaximumWidth(width());

    QPoint p(0, height());
    int x = mapToGlobal(p).x();
    int y = mapToGlobal(p).y() + 1;

    int hits = list_view->model()->rowCount();
    int h = std::min(6, hits) * line_height * 2 + 4;

    if(hits == 0) {
        list_view->hide();

    } else {
        list_view->setCurrentIndex(list_view->model()->index(0,0));

        list_view->move(x, y);
        list_view->setFixedHeight(h);

        list_view->show();
    }
}

void CompleteLineEdit::completeText(const QModelIndex &index) {
    QString text = index.data().toString();
    setText(text);
    list_view->hide();
    Q_EMIT editingFinished();
}

BoxDialog::BoxDialog(QString message, NodeFactory& node_factory, NodeAdapterFactory &adapter_factory, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f), node_factory_(node_factory), adapter_factory_(adapter_factory),
      message_(message)
{
    makeUI();
}

void BoxDialog::makeUI()
{
    setWindowIcon(QIcon(":/add_node.png"));
    setWindowTitle("Create Node");

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
            NodeListGenerator generator(node_factory_, adapter_factory_);
            model_ = generator.listAvailableNodeTypes();

            Q_EMIT pluginsLoaded();

            return true;
        });
    }
}

void BoxDialog::setupTextBox()
{
    name_edit_ = new CompleteLineEdit;
    name_edit_->setFixedWidth(350);

    filter = new NodeFilterProxyModel;
    filter->setFilterCaseSensitivity(Qt::CaseInsensitive);

    QObject::connect(name_edit_, SIGNAL(textChanged(QString)), filter, SLOT(setFilterFixedString(const QString &)));
    QObject::connect(name_edit_, SIGNAL(textChanged(QString)), filter, SLOT(invalidate()));

    name_edit_->setModel(filter);

    delete loading_;
    layout()->addWidget(name_edit_);

    name_edit_->setFocus();

    filter->setSourceModel(model_);

    QObject::connect(name_edit_, SIGNAL(editingFinished()), this, SLOT(finish()));
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

std::string BoxDialog::getName()
{
    return name_edit_->text().toStdString();
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_box_dialog.cpp"
