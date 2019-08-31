/// HEADER
#include <csapex/view/widgets/completed_line_edit.h>

/// PROJECT
#include <csapex/view/utility/html_delegate.h>
#include <csapex/csapex_mime.h>

/// SYSTEM
#include <QListView>
#include <QKeyEvent>

using namespace csapex;

CompletedLineEdit::CompletedLineEdit(QWidget* parent) : QLineEdit(parent), was_hidden(false)
{
    list_view = new QListView(this);
    list_view->setWindowFlags(Qt::ToolTip);

    HTMLBoxDelegate* delegate = new HTMLBoxDelegate();
    list_view->setItemDelegate(delegate);

    connect(list_view, SIGNAL(clicked(const QModelIndex&)), this, SLOT(completeText(const QModelIndex&)));
    connect(this, SIGNAL(textChanged(QString)), delegate, SLOT(setKeyWords(const QString&)));
}

void CompletedLineEdit::focusOutEvent(QFocusEvent* /*e*/)
{
    list_view->hide();
    was_hidden = true;
}

void CompletedLineEdit::focusInEvent(QFocusEvent* /*e*/)
{
    // only show the view, if it has been visible before and then has been hidden!
    if (was_hidden) {
        list_view->show();
        update();
    }
}

void CompletedLineEdit::keyPressEvent(QKeyEvent* e)
{
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
                completeText(currentIndex);
            } else {
                completeText(list_view->rootIndex());
            }

            list_view->hide();

        } else {
            QLineEdit::keyPressEvent(e);
        }

    } else {
        QLineEdit::keyPressEvent(e);
    }
}

void CompletedLineEdit::setModel(QAbstractItemModel* model)
{
    list_view->setModel(model);

    QObject::connect(model, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(update()));
    QObject::connect(model, SIGNAL(layoutChanged()), this, SLOT(update()));
}

void CompletedLineEdit::update()
{
    list_view->setMinimumWidth(width());
    list_view->setMaximumWidth(width());

    int hits = list_view->model()->rowCount();

    const int max_items_to_show = 6;
    const int visible_items = std::min(max_items_to_show, hits);

    const auto visible_height = visible_items * list_view->sizeHintForRow(0);

    list_view->setMinimumHeight(visible_height);
    list_view->setMaximumHeight(visible_height);

    QPoint p(0, height());
    int x = mapToGlobal(p).x();
    int y = mapToGlobal(p).y() + 1;

    if (hits == 0) {
        list_view->hide();

    } else {
        list_view->setCurrentIndex(list_view->model()->index(0, 0));

        list_view->move(x, y);

        list_view->show();
    }
}

void CompletedLineEdit::completeText(const QModelIndex& index)
{
    mime_ = index.data(Qt::UserRole + 5).toString().toStdString();

    QString text = index.data().toString();
    setText(text);

    list_view->hide();

    Q_EMIT editingFinished();
}

std::string CompletedLineEdit::getMIME() const
{
    return mime_;
}

std::string CompletedLineEdit::getName() const
{
    return text().toStdString();
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_completed_line_edit.cpp"
