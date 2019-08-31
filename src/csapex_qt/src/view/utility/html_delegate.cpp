/// HEADER
#include <csapex/view/utility/html_delegate.h>

/// SYSTEM
#include <QPainter>
#include <QTextDocument>
#include <QApplication>
#include <QAbstractTextDocumentLayout>
#include <iostream>

using namespace csapex;

HTMLDelegate::HTMLDelegate()
{
}

QSize HTMLDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QStyleOptionViewItem options = option;
    initStyleOption(&options, index);

    options.rect.setSize(QStyledItemDelegate::sizeHint(option, index));

    QTextDocument doc;
    doc.setTextWidth(options.rect.width());
    doc.setDefaultFont(options.font);
    doc.setHtml(options.text);
    doc.adjustSize();

    return doc.size().toSize();
}

void HTMLDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QStyleOptionViewItem options = option;
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

HTMLBoxDelegate::HTMLBoxDelegate() : HTMLDelegate()
{
}

void HTMLBoxDelegate::setKeyWords(const QString& words)
{
    key_words = words.split(QRegExp("(\\s+|::)"));
}

QString HTMLBoxDelegate::createText(const QModelIndex& index) const
{
    QString descr = index.data(Qt::UserRole + 1).toString();
    QString name = index.data(Qt::UserRole + 2).toString();
    QStringList tags = index.data(Qt::UserRole + 3).toStringList();
    QStringList properties = index.data(Qt::UserRole + 4).toStringList();

    QString tag;
    if (!tags.empty()) {
        tag = tags.at(0);
        for (const QString& t : tags) {
            for (const QString& s : key_words) {
                if (s.length() > 0) {
                    if (t.contains(s, Qt::CaseInsensitive)) {
                        tag = t;
                    }
                }
            }
        }
    }

    for (const QString& s : key_words) {
        if (s.length() > 0 && s != ".") {
            descr.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b><u>\\1</u></b>");
            name.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b><u>\\1</u></b>");
            tag.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<b><u>\\1</u></b>");
        }
    }

    QString properties_str;
    bool invalid = false;
    for (QString property : properties) {
        if (property == "invalid") {
            invalid = true;
        }
        for (const QString& s : key_words) {
            if (property.contains(s, Qt::CaseInsensitive)) {
                property.replace(QRegExp(QString("(") + s + ")", Qt::CaseInsensitive), "<span style='color: #000'><u>\\1</u></span>");
            }
        }

        properties_str += property + " &nbsp; ";
    }

    QString html;
    html += "<span style='" + (invalid ? QString("color: #f00") : QString("")) + "'><b>";
    if (!tag.isEmpty()) {
        html += "<small>" + tag + " :: </small>";
    }
    html += name + "</b></span> &nbsp; ";
    html += "<small><span style='color: " + (invalid ? QString("#f00") : QString("#888")) + "'>";
    html += properties_str;
    html += "</span></small>";
    html += "<br /><small><i>" + descr + "</i></small>";
    return html;
}

void HTMLBoxDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QStyleOptionViewItem options = option;
    initStyleOption(&options, index);

    painter->save();

    QTextDocument doc;
    // Set the text width so large to effectively disable wrapping
    doc.setTextWidth(options.rect.width() * 30);
    doc.setDefaultFont(options.font);
    doc.setHtml(createText(index));

    options.text = "";
    options.widget->style()->drawControl(QStyle::CE_ItemViewItem, &options, painter);

    painter->translate(options.rect.left() + 18, options.rect.top());
    QRect clip(0, 0, options.rect.width(), options.rect.height());
    doc.drawContents(painter, clip);

    painter->restore();
}

QSize HTMLBoxDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QStyleOptionViewItem options = option;
    initStyleOption(&options, index);

    QTextDocument doc;
    doc.setTextWidth(options.rect.width());
    doc.setDefaultFont(options.font);
    doc.setHtml(option.text);
    doc.adjustSize();

    // We are rendering two lines of text
    auto height = option.fontMetrics.height() * 2;

    return QSize(doc.idealWidth(), height);
}
/// MOC
#include "../../../include/csapex/view/utility/moc_html_delegate.cpp"
