/// HEADER
#include <csapex/view/model/thread_group_table_model.h>

/// PROJECT
#include <csapex/scheduling/thread_pool.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/command/modify_thread.h>
#include <csapex/command/command_executor.h>
#include <csapex/core/settings.h>

using namespace csapex;

ThreadGroupTableModel::ThreadGroupTableModel(Settings& settings, ThreadPoolPtr thread_pool, CommandExecutor& dispatcher)
    : settings_(settings), thread_pool_(thread_pool), cmd_executor_(dispatcher)
{
    observe(settings_.setting_changed, [this](const std::string& name) {
        if(name == "debug") {
            refresh();
        }
    });

    observe(thread_pool_->begin_step, [this](){
        refresh();
    });
    observe(thread_pool_->end_step, [this](){
        refresh();
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

int ThreadGroupTableModel::rowCount(const QModelIndex &/*parent*/) const
{
    return thread_pool_->getGroupCount() + 1;
}

int ThreadGroupTableModel::columnCount(const QModelIndex &/*parent*/) const
{
    return settings_.get<bool>("debug") ? 3 : 2;
}

ThreadGroup * ThreadGroupTableModel::getThreadGroup(int row) const
{
    return row == 0 ? thread_pool_->getDefaultGroup() : thread_pool_->getGroupAt(row - 1);
}

bool ThreadGroupTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    ThreadGroup* group = getThreadGroup(index.row());

    switch(index.column()) {
    case 0:
    {
        QString var = value.toString();
//        group->setName(var.toStdString());

        command::ModifyThread::Ptr modify(new command::ModifyThread(group->id(), var.toStdString()));
        cmd_executor_.execute(modify);
    }
        break;
    default:
        break;
    }

    QModelIndex top = createIndex(index.row(), 0);
    QModelIndex bottom = createIndex(index.row(), 5);

    dataChanged(top, bottom);

    return true;
}

QVariant ThreadGroupTableModel::data(const QModelIndex &index, int role) const
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
        {
            QString res;
            res += "(";
            res += QString::number(group->isStepDone());
            res += ", ";
            res += QString::number(group->canStartStepping());
            res += ")";
            return res;
        }
        default:
            break;
        }
    }
        break;
    }

    return QVariant();
}



QVariant ThreadGroupTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::ToolTipRole) {
        if(orientation == Qt::Horizontal) {
            switch(section) {
            case 2: return "isStepDone(), canStartStepping()";
            default:
                break;
            }
        }
        return QVariant();
    }
    if (role != Qt::DisplayRole) {
        return QVariant();
    }

    if(orientation == Qt::Horizontal) {
        switch(section) {
        case 0: return "Name";
        case 1: return "Nodes";
        case 2: return "States";
        default:
            break;
        }

    } else {
        return section + 1;
    }

    return QVariant();
}

Qt::ItemFlags ThreadGroupTableModel::flags(const QModelIndex &index) const
{
    Qt::ItemFlags flags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;

    if(index.column() == 0) {
        flags |= Qt::ItemIsEditable;
    }

    return flags;
}


void ThreadGroupTableModel::refresh()
{
    beginResetModel();
    endResetModel();
}
