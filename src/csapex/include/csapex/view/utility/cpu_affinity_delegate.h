#ifndef CPU_AFFINITY_DELEGATE_H
#define CPU_AFFINITY_DELEGATE_H

/// SYSTEM
#include <QStyledItemDelegate>

namespace csapex
{
class CpuAffinityDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    CpuAffinityDelegate(QWidget *parent = 0);

    void paint(QPainter *painter, const QStyleOptionViewItem &option,
               const QModelIndex &index) const override;
    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const override;
    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                          const QModelIndex &index) const override;
    void setEditorData(QWidget *editor, const QModelIndex &index) const override;
    void setModelData(QWidget *editor, QAbstractItemModel *model,
                      const QModelIndex &index) const override;

private slots:
    void commitAndCloseEditor();
};
}

#endif // CPU_AFFINITY_DELEGATE_H
