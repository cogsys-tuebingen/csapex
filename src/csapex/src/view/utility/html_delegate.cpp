/// HEADER
#include <csapex/view/utility/html_delegate.h>

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

    options.rect.setSize(QStyledItemDelegate::sizeHint(option, index));

    QTextDocument doc;
    doc.setTextWidth(options.rect.width());
    doc.setDefaultFont(options.font);
    doc.setHtml(options.text);
    doc.adjustSize();

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
    doc.adjustSize();

    options.text = "";
    options.widget->style()->drawControl(QStyle::CE_ItemViewItem, &options, painter);

    int dx = options.icon.isNull() ? 0 : 18;
    int dy = 0;

    options.rect.setX(options.rect.x() + dx);
    options.rect.setY(options.rect.y() + dy);

    painter->translate(options.rect.left(), options.rect.top());
    QRect clip(0, 0, options.rect.width(), options.rect.height());
    doc.drawContents(painter, clip);

    painter->restore();
}


HTMLBoxDelegate::HTMLBoxDelegate(int line_height)
    : HTMLDelegate(line_height)
{
}

void HTMLBoxDelegate::setKeyWords (const QString& words)
{
    key_words = words.split(QRegExp("(\\s+|::)"));
}
void HTMLBoxDelegate::paint ( QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index ) const
{
    QStyleOptionViewItemV4 options = option;
    initStyleOption(&options, index);

    painter->save();

    QString descr = index.data(Qt::UserRole + 1).toString();
    QString name = index.data(Qt::UserRole + 2).toString();
    QStringList tags = index.data(Qt::UserRole + 3).toStringList();
    QStringList properties = index.data(Qt::UserRole + 4).toStringList();

    QString tag;
    if(!tags.empty()) {
        tag =  tags.at(0);
        for(const QString& t : tags) {
            for(const QString& s : key_words) {
                if(s.length() > 0) {
                    if(t.contains(s, Qt::CaseInsensitive)) {
                        tag = t;
                    }
                }
            }
        }

    }

    for(const QString& s : key_words) {
        if(s.length() > 0 && s != ".") {
            descr.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b><u>\\1</u></b>");
            name.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b><u>\\1</u></b>");
            tag.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b><u>\\1</u></b>");
        }
    }


    QString properties_str;
    bool invalid = false;
    for(QString property : properties) {
        if(property == "invalid") {
            invalid = true;
        }
        for(const QString& s : key_words) {
            if(property.contains(s, Qt::CaseInsensitive)) {
                property.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<span style='color: #000'><u>\\1</u></span>");
            }
        }

        properties_str += property + " &nbsp; ";
    }

    QTextDocument doc;
    QString html;
    html += "<table><tr><th" +
            (invalid ? QString(" style='color: #f00'") : QString("") ) +
            ">";
    if(!tag.isEmpty()) {
        html += "<small>" + tag + " :: </small>" ;
    }
    html += name + "</th>";
    html += "<th style='font-size: 10px; color: " +
            (invalid ? QString("#f00") : QString("#888") ) +
            "; padding-left: 6px' valign='middle'>";
    html += properties_str;
    html += "</th>";
    html += "</tr></table>";
    html += "<br /><small><i>" + descr + "</i></small>";

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
#include "../../../include/csapex/view/utility/moc_html_delegate.cpp"
