#ifndef HTML_DELEGATE_H
#define HTML_DELEGATE_H

/// COMPONENT
#include <csapex_qt/export.h>

/// SYSTEM
#include <QStyledItemDelegate>

namespace csapex
{
class CSAPEX_QT_EXPORT HTMLDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    HTMLDelegate();

protected:
    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const;

    QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const;
};

class CSAPEX_QT_EXPORT HTMLBoxDelegate : public HTMLDelegate
{
    Q_OBJECT

public:
    HTMLBoxDelegate();

public Q_SLOTS:
    void setKeyWords(const QString& words);

protected:
    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const;
    QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const;

    QString createText(const QModelIndex& index) const;

    QStringList key_words;
};

}  // namespace csapex
#endif  // HTML_DELEGATE_H
