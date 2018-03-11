#ifndef THREAD_GROUP_PROFILING_DELEGATE_H
#define THREAD_GROUP_PROFILING_DELEGATE_H

/// SYSTEM
#include <QStyledItemDelegate>

namespace csapex
{
class ThreadGroupProfilingDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    ThreadGroupProfilingDelegate(QWidget *parent = 0);

    void paint(QPainter *painter, const QStyleOptionViewItem &option,
               const QModelIndex &index) const override;
    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const override;
};
}

#endif // THREAD_GROUP_PROFILING_DELEGATE_H
