/// HEADER
#include <csapex/view/box_dialog.h>

/// COMPONENT
#include <csapex/view/widget_controller.h>
#include <csapex/model/node_filter_proxy_model.h>
#include <csapex/utility/html_delegate.h>

/// SYSTEM
#include <QLabel>
#include <QKeyEvent>
#include <QListView>

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

BoxDialog::BoxDialog(WidgetController* widget_controller, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f), widget_controller_(widget_controller)
{
    makeUI();
}

void BoxDialog::makeUI()
{
    setWindowIcon(QIcon(":/add_node.png"));
    setWindowTitle("Create Node");

    setModal(true);
    setWindowFlags(Qt::WindowStaysOnTopHint);

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    QLabel *lbl = new QLabel("<img src=\":/add_node.png\"> Please enter the type of node to add. (<em>Autocompleted</em>)");
    name_edit_ = new CompleteLineEdit;
    name_edit_->setFixedWidth(350);
    lbl->setBuddy(name_edit_);

    NodeFilterProxyModel* filter = new NodeFilterProxyModel;
    filter->setFilterCaseSensitivity(Qt::CaseInsensitive);
    filter->setSourceModel(widget_controller_->listAvailableNodeTypes());

    connect(name_edit_, SIGNAL(textChanged(QString)), filter, SLOT(setFilterFixedString(const QString &)));
    connect(name_edit_, SIGNAL(textChanged(QString)), filter, SLOT(invalidate()));

    name_edit_->setModel(filter);

    layout->addWidget(lbl);
    layout->addWidget(name_edit_);

    connect(name_edit_, SIGNAL(editingFinished()), this, SLOT(finish()));
}

void BoxDialog::finish()
{
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
#include "../../include/csapex/view/moc_box_dialog.cpp"
