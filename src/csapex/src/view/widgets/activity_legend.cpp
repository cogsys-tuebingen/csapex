/// HEADER
#include <csapex/view/widgets/activity_legend.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>

/// SYSTEM
#include <QHeaderView>

using namespace csapex;

ActivityLegend::ActivityLegend()
{
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    setMinimumSize(10, 10);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    model()->insertColumn(0);
    horizontalHeader()->setHidden(true);
    verticalHeader()->setHidden(true);

    QObject::connect(this, SIGNAL(itemSelectionChanged()), this, SLOT(emitSelection()));
}

void ActivityLegend::resizeToFit()
{
    resizeColumnsToContents();

    QRect rect = geometry();
    int tableWidth = 2 + verticalHeader()->width();
    for(int i = 0; i < columnCount(); i++){
        tableWidth += columnWidth(i);
    }
    rect.setWidth(tableWidth);
    int tableHeight = 2 + horizontalHeader()->height();
    for(int i = 0; i < rowCount(); i++){
        tableHeight += rowHeight(i);
    }
    rect.setHeight(tableHeight);

    setFixedWidth(rect.width());
    setFixedHeight(rect.height());
}

void ActivityLegend::startTrackingNode(NodeWorkerPtr node)
{
    NodeWorker* worker = node.get();

    worker->start_profiling.connect([this](NodeWorker* nw) { addNode(nw); });
    worker->stop_profiling.connect([this](NodeWorker* nw) { removeNode(nw); });
}

void ActivityLegend::stopTrackingNode(NodeHandlePtr /*node*/)
{
}

void ActivityLegend::addNode(NodeWorker* node)
{
    int row = rows_.size();
    rows_.push_back(node);

    QAbstractItemModel* m = model();
    m->insertRow(row);
    m->setData(m->index(row, 0), QString::fromStdString(node->getNodeHandle()->getNodeState()->getLabel()));

    resizeToFit();

    Q_EMIT nodeAdded(node);
}

void ActivityLegend::removeNode(NodeWorker* node)
{
    bool found = false;
    int row = 0;
    for(std::size_t r = 0; r < rows_.size(); ++r) {
        if(found) {
            // deleted -> move one up
            rows_[r-1] = rows_[r];
        }
        if(rows_[r] == node) {
            row = r;
            found = true;
        }
    }

    if(found) {
        removeRow(row);
        rows_.pop_back();

        resizeToFit();

        Q_EMIT nodeRemoved(node);
    }
}

void ActivityLegend::emitSelection()
{
    QList<NodeWorker*> list;

    for(QModelIndex i : selectedIndexes()) {
        list.push_back(rows_[i.row()]);
    }

    Q_EMIT nodeSelectionChanged(list);
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_activity_legend.cpp"
