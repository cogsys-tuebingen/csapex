#ifndef THREAD_GROUP_PROFILING_MODEL_H
#define THREAD_GROUP_PROFILING_MODEL_H

/// PROJECT
#include <csapex/command/command_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/core/core_fwd.h>

/// SYTEM
#include <QAbstractTableModel>

namespace csapex
{

class ThreadGroupProfilingModel : public QAbstractTableModel, public Observer
{
public:
    ThreadGroupProfilingModel(Settings &settings, ThreadPoolPtr thread_pool);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    ThreadGroup * getThreadGroup(int row) const;

    void refresh();

private:
    Settings& settings_;
    ThreadPoolPtr thread_pool_;
};

}

#endif // THREAD_GROUP_PROFILING_MODEL_H
