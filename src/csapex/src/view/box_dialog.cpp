/// HEADER
#include <csapex/view/box_dialog.h>

/// COMPONENT
#include <csapex/manager/box_manager.h>
#include <csapex/model/node_filter_proxy_model.h>

/// SYSTEM
#include <QIcon>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QAbstractItemModel>
#include <QKeyEvent>
#include <QtGui/QListView>
#include <QtGui/QStringListModel>
#include <QDebug>
#include <QPainter>
#include <QTextDocument>

using namespace csapex;

HTMLDelegate::HTMLDelegate(int line_height)
    : line_height(line_height)
{
}

void HTMLDelegate::setKeyWords (const QString& words) {
    key_words = words.split(QRegExp("(\\s+|::)"));
}

void HTMLDelegate::paint ( QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index ) const {
    QStyleOptionViewItemV4 options = option;
    initStyleOption(&options, index);

    painter->save();

    QString descr = index.data(Qt::UserRole + 1).toString();
    QString name = index.data(Qt::UserRole + 2).toString();
    QStringList tags = index.data(Qt::UserRole + 3).toStringList();

    if(tags.empty()) {
        return;
    }

    QString tag = tags.at(0);
    Q_FOREACH(const QString& t, tags) {
        Q_FOREACH(const QString& s, key_words) {
            if(s.length() > 0) {
                if(t.contains(s, Qt::CaseInsensitive)) {
                    tag = t;
                }
            }
        }
    }

    Q_FOREACH(const QString& s, key_words) {
        if(s.length() > 0) {
            descr.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b>\\1</b>");
            name.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b>\\1</b>");
            tag.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b>\\1</b>");
        }
    }

    QTextDocument doc;
    QString html = QString("<small>") + tag + " :: </small>" + name + "<br /><small><i>" + descr + "</i></small>";

    doc.setHtml(html);

    options.text = "";
    options.widget->style()->drawControl(QStyle::CE_ItemViewItem, &options, painter);

    painter->translate(options.rect.left() + 18, options.rect.top());
    QRect clip(0, 0, options.rect.width(), options.rect.height());
    doc.drawContents(painter, clip);

    painter->restore();
}


QSize HTMLDelegate::sizeHint ( const QStyleOptionViewItem & option, const QModelIndex & index ) const {
    QStyleOptionViewItemV4 options = option;
    initStyleOption(&options, index);

    QTextDocument doc;
    doc.setHtml(options.text);
    doc.setTextWidth(options.rect.width());
    return QSize(doc.idealWidth(), 2 * line_height);
}

CompleteLineEdit::CompleteLineEdit(QWidget *parent)
    : QLineEdit(parent)
{
    line_height = 20;

    list_view = new QListView(this);
    list_view->setWindowFlags(Qt::ToolTip);

    HTMLDelegate* delegate = new HTMLDelegate(line_height);
    list_view->setItemDelegate(delegate);

    connect(list_view, SIGNAL(clicked(const QModelIndex&)), this, SLOT(completeText(const QModelIndex&)));
    connect(this, SIGNAL(textChanged(QString)), delegate, SLOT(setKeyWords(const QString&)));
    connect(this, SIGNAL(lostFocus()), list_view, SLOT(hide()));
}


void CompleteLineEdit::focusOutEvent(QFocusEvent */*e*/) {
    Q_EMIT editingFinished();
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

BoxDialog::BoxDialog(QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f)
{
    makeUI();
}

void BoxDialog::makeUI()
{
    setWindowIcon(QIcon(":/add_node.png"));
    setWindowTitle("Create Node");

    setModal(true);

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    QLabel *lbl = new QLabel("<img src=\":/add_node.png\"> Please enter the type of node to add. (<em>Autocompleted</em>)");
    name_edit_ = new CompleteLineEdit;
    name_edit_->setFixedWidth(350);
    lbl->setBuddy(name_edit_);

    NodeFilterProxyModel* filter = new NodeFilterProxyModel;
    filter->setFilterCaseSensitivity(Qt::CaseInsensitive);
    filter->setSourceModel(BoxManager::instance().listAvailableBoxedObjects());

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

