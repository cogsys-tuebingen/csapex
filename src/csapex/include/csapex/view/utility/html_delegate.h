#ifndef HTML_DELEGATE_H
#define HTML_DELEGATE_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QStyledItemDelegate>

namespace csapex
{
class CSAPEX_QT_EXPORT HTMLDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    HTMLDelegate(int line_height = -1);

protected:
    void paint ( QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index ) const;

    QSize sizeHint ( const QStyleOptionViewItem & option, const QModelIndex & index ) const;

    int line_height;
};

class CSAPEX_QT_EXPORT HTMLBoxDelegate : public HTMLDelegate
{
    Q_OBJECT

public:
    HTMLBoxDelegate(int line_height = -1);

public Q_SLOTS:
    void setKeyWords (const QString& words);

protected:
    void paint ( QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index ) const;
    QSize sizeHint ( const QStyleOptionViewItem & option, const QModelIndex & index ) const;

    QStringList key_words;
};

}
#endif // HTML_DELEGATE_H
