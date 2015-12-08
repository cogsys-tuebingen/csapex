/// HEADER
#include <csapex/utility/html_delegate.h>

/// SYSTEM
#include <QPainter>
#include <QTextDocument>
#include <iostream>

using namespace csapex;

HTMLDelegate::HTMLDelegate(int line_height)
    : line_height(line_height)
{

}



QSize HTMLDelegate::sizeHint ( const QStyleOptionViewItem & option, const QModelIndex & index ) const
{
    QStyleOptionViewItemV4 options = option;
    initStyleOption(&options, index);

    QTextDocument doc;
    doc.setTextWidth(options.rect.width());
    doc.setDefaultFont(options.font);
    doc.setHtml(options.text);

    return doc.size().toSize();
}


void HTMLDelegate::paint ( QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index ) const
{
    QStyleOptionViewItemV4 options = option;
    initStyleOption(&options, index);

    painter->save();

    QTextDocument doc;
    QString html = options.text;
    doc.setTextWidth(options.rect.width());
    doc.setDefaultFont(options.font);
    doc.setHtml(html);
    options.text = "";
    options.widget->style()->drawControl(QStyle::CE_ItemViewItem, &options, painter);

    painter->translate(options.rect.left(), options.rect.top());
    QRect clip(0, 0, options.rect.width(), options.rect.height());
    doc.drawContents(painter, clip);

    painter->restore();
}


HTMLBoxDelegate::HTMLBoxDelegate(int line_height)
    : HTMLDelegate(line_height)
{
}

void HTMLBoxDelegate::setKeyWords (const QString& words) {
    key_words = words.split(QRegExp("(\\s+|::)"));
}

void HTMLBoxDelegate::paint ( QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index ) const {
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




QSize HTMLBoxDelegate::sizeHint ( const QStyleOptionViewItem & option, const QModelIndex & index ) const
{
    QStyleOptionViewItemV4 options = option;
    initStyleOption(&options, index);

    QTextDocument doc;
    doc.setHtml(options.text);
    doc.setTextWidth(options.rect.width());
    return QSize(doc.idealWidth(), 2 * line_height);
}
/// MOC
#include "../../include/csapex/utility/moc_html_delegate.cpp"
