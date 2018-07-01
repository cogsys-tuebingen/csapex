/// HEADER
#include <csapex/view/utility/thread_group_profiling_delegate.h>

/// PROJECT
#include <csapex/view/widgets/thread_group_profiling_widget.h>
#include <csapex/view/model/thread_group_profiling_model.h>
#include <csapex/scheduling/thread_group.h>

/// SYSTEM
#include <QtWidgets>
#include <iostream>

using namespace csapex;

ThreadGroupProfilingDelegate::ThreadGroupProfilingDelegate(QWidget* parent) : QStyledItemDelegate(parent)
{
}

void ThreadGroupProfilingDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    if (index.data().canConvert<ThreadGroupProfilingRenderer>()) {
        ThreadGroupProfilingRenderer thread_group_profiler = qvariant_cast<ThreadGroupProfilingRenderer>(index.data());

        if (option.state & QStyle::State_Selected)
            painter->fillRect(option.rect, option.palette.highlight());

        QWidget* parentw = dynamic_cast<QWidget*>(parent());
        QTableView* tv = dynamic_cast<QTableView*>(parentw);
        int dx = 0;
        int dy = 0;
        if (tv) {
            dx = -tv->verticalHeader()->width();
            dy = -tv->horizontalHeader()->height();
        }
        QPoint pos = parentw->mapFromGlobal(parentw->cursor().pos() + QPoint(dx, dy));

        thread_group_profiler.paint(painter, option.rect, option.palette, pos);

    } else {
        QStyleOptionViewItem opt = option;

        const ThreadGroupProfilingModel* model = dynamic_cast<const ThreadGroupProfilingModel*>(index.model());
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

QSize ThreadGroupProfilingDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    if (index.data().canConvert<ThreadGroupProfilingRenderer>()) {
        ThreadGroupProfilingRenderer profiling_renderer = qvariant_cast<ThreadGroupProfilingRenderer>(index.data());
        return profiling_renderer.sizeHint();
    } else {
        return QStyledItemDelegate::sizeHint(option, index);
    }
}

/// MOC
#include "../../../include/csapex/view/utility/moc_thread_group_profiling_delegate.cpp"
