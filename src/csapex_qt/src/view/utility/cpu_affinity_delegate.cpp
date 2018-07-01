/// HEADER
#include <csapex/view/utility/cpu_affinity_delegate.h>

/// PROJECT
#include <csapex/utility/cpu_affinity.h>
#include <csapex/view/widgets/cpu_affinity_widget.h>
#include <csapex/view/model/thread_group_table_model.h>
#include <csapex/scheduling/thread_group.h>

/// SYSTEM
#include <QtWidgets>

using namespace csapex;

CpuAffinityDelegate::CpuAffinityDelegate(QWidget* parent) : QStyledItemDelegate(parent)
{
}

void CpuAffinityDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    if (index.data().canConvert<CpuAffinityRenderer>()) {
        CpuAffinityRenderer cpu_affinity = qvariant_cast<CpuAffinityRenderer>(index.data());

        if (option.state & QStyle::State_Selected)
            painter->fillRect(option.rect, option.palette.highlight());

        cpu_affinity.paint(painter, option.rect, option.palette, CpuAffinityRenderer::ReadOnly);
    } else {
        QStyleOptionViewItem opt = option;

        const ThreadGroupTableModel* model = dynamic_cast<const ThreadGroupTableModel*>(index.model());
        apex_assert_hard(model);

        ThreadGroup* group = model->getThreadGroup(index.row());
        apex_assert_hard(group);

        if (group->id() == ThreadGroup::PRIVATE_THREAD) {
            opt.font.setItalic(true);
        } else {
            opt.font.setBold(true);
        }
        QStyledItemDelegate::paint(painter, opt, index);
    }
}

QSize CpuAffinityDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    if (index.data().canConvert<CpuAffinityRenderer>()) {
        CpuAffinityRenderer cpu_affinity = qvariant_cast<CpuAffinityRenderer>(index.data());
        return cpu_affinity.sizeHint();
    } else {
        return QStyledItemDelegate::sizeHint(option, index);
    }
}

QWidget* CpuAffinityDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const

{
    if (index.data().canConvert<CpuAffinityRenderer>()) {
        const ThreadGroupTableModel* model = dynamic_cast<const ThreadGroupTableModel*>(index.model());
        apex_assert_hard(model);

        ThreadGroup* group = model->getThreadGroup(index.row());
        apex_assert_hard(group);

        CpuAffinityWidget* editor = new CpuAffinityWidget(group->getCpuAffinity(), parent);
        //        connect(editor, &CpuAffinityWidget::editingFinished,
        //                this, &CpuAffinityDelegate::commitAndCloseEditor);
        connect(editor, &CpuAffinityWidget::destroyed, this, &CpuAffinityDelegate::commitAndCloseEditor);
        return editor;
    } else {
        return QStyledItemDelegate::createEditor(parent, option, index);
    }
}

void CpuAffinityDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    if (index.data().canConvert<CpuAffinityRenderer>()) {
        CpuAffinityRenderer cpu_affinity = qvariant_cast<CpuAffinityRenderer>(index.data());
        CpuAffinityWidget* starEditor = qobject_cast<CpuAffinityWidget*>(editor);
        starEditor->setRenderer(cpu_affinity);
    } else {
        QStyledItemDelegate::setEditorData(editor, index);
    }
}

void CpuAffinityDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    if (index.data().canConvert<CpuAffinityRenderer>()) {
        CpuAffinityWidget* starEditor = qobject_cast<CpuAffinityWidget*>(editor);
        model->setData(index, QVariant::fromValue(starEditor->getWidget()));
    } else {
        QStyledItemDelegate::setModelData(editor, model, index);
    }
}

void CpuAffinityDelegate::commitAndCloseEditor()
{
    CpuAffinityWidget* editor = qobject_cast<CpuAffinityWidget*>(sender());
    emit commitData(editor);
    emit closeEditor(editor);
}

/// MOC
#include "../../../include/csapex/view/utility/moc_cpu_affinity_delegate.cpp"
