/// HEADER
#include <csapex/view/model/thread_group_profiling_model.h>

/// PROJECT
#include <csapex/scheduling/thread_pool.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/core/settings.h>
#include <csapex/view/widgets/thread_group_profiling_widget.h>

/// SYSTEM
#include <QTimer>

using namespace csapex;

ThreadGroupProfilingModel::ThreadGroupProfilingModel(Settings& settings, ThreadPoolPtr thread_pool)
    : settings_(settings), thread_pool_(thread_pool)
{
    observe(settings_.setting_changed, [this](const std::string& name) {
        if(name == "debug") {
            refresh();
        }
    });

    observe(thread_pool_->group_created, [this](ThreadGroupPtr group){
        refresh();

        observe(group->scheduler_changed, [this](){
            refresh();
        });

        observe(group->generator_added, [this](TaskGeneratorPtr){
            refresh();
        });

        observe(group->generator_removed, [this](TaskGeneratorPtr){
            refresh();
        });
    });

    observe(thread_pool_->group_removed, [this](ThreadGroupPtr group){
        refresh();
    });
}

int ThreadGroupProfilingModel::rowCount(const QModelIndex &/*parent*/) const
{
    return thread_pool_->getGroupCount();
}

int ThreadGroupProfilingModel::columnCount(const QModelIndex &/*parent*/) const
{
    return 3;
}

ThreadGroup * ThreadGroupProfilingModel::getThreadGroup(int row) const
{
    return thread_pool_->getGroupAt(row);
}

QVariant ThreadGroupProfilingModel::data(const QModelIndex &index, int role) const
{
    ThreadGroup* group = getThreadGroup(index.row());

    switch(role) {
    case Qt::DisplayRole:
    case Qt::EditRole:
    {
        switch(index.column()) {
        case 0:
            return QString::fromStdString(group->getName());
        case 1:
            return QVariant::fromValue(group->size());
        case 2:
            return QVariant::fromValue(ThreadGroupProfilingRenderer(group));

        default:
            break;
        }
    }
        break;
    case Qt::ToolTipRole:
    {
        switch(index.column()) {
        case 0:
            return QString::fromStdString(group->getName());
        case 1:
            return QVariant::fromValue(group->size());
        case 2:
        {
            auto& highlights = ThreadGroupProfilingRendererGlobalState::instance().highlight;
            auto pos = highlights.find(group);
            if(pos != highlights.end()) {
                return QString::fromStdString(pos->second);
            }
        }
        default:
            break;
        }
    }
        break;
    }
    return QVariant();
}



QVariant ThreadGroupProfilingModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole) {
        return QVariant();
    }
    if(orientation == Qt::Horizontal) {
        switch(section) {
        case 0: return "Name";
        case 1: return "Nodes";
        case 2: return "Profiling";
        default:
            break;
        }

    } else {
        return section + 1;
    }

    return QVariant();
}

Qt::ItemFlags ThreadGroupProfilingModel::flags(const QModelIndex &index) const
{
    Qt::ItemFlags flags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;
    return flags;
}


void ThreadGroupProfilingModel::refresh()
{
    beginResetModel();
    endResetModel();
}
