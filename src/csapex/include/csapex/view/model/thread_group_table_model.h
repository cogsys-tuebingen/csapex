#ifndef THREAD_GROUP_TABLE_MODEL_H
#define THREAD_GROUP_TABLE_MODEL_H

/// PROJECT
#include <csapex/command/command_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/core/core_fwd.h>

/// SYTEM
#include <QAbstractTableModel>

namespace csapex
{

class ThreadGroupTableModel : public QAbstractTableModel, public Observer
{
public:
    ThreadGroupTableModel(Settings &settings, ThreadPoolPtr thread_pool, CommandExecutor& dispatcher);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    ThreadGroup * getThreadGroup(int row) const;

private:
    void refresh();

private:
    Settings& settings_;
    ThreadPoolPtr thread_pool_;
    CommandExecutor& cmd_executor_;
};

}

#endif // THREAD_GROUP_TABLE_MODEL_H
