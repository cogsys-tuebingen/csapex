/// HEADER
#include <csapex/view/node/node_filter_proxy_model.h>

/// SYSTEM
#include <QStringList>

using namespace csapex;

NodeFilterProxyModel::NodeFilterProxyModel(QObject *parent)
    : QSortFilterProxyModel(parent)
{
}

bool NodeFilterProxyModel::filterAcceptsRow(int sourceRow, const QModelIndex &) const
{
    QString descr = sourceModel()->index(sourceRow, 0).data(Qt::UserRole + 1).toString();
    QString name = sourceModel()->index(sourceRow, 0).data(Qt::UserRole + 2).toString();
    QStringList tags = sourceModel()->index(sourceRow, 0).data(Qt::UserRole + 3).toStringList();
    QStringList properties = sourceModel()->index(sourceRow, 0).data(Qt::UserRole + 4).toStringList();

    QStringList qrys = filterRegExp().pattern().split(" ", QString::SkipEmptyParts);

    bool contains_all = true;
    for(const QString& qry : qrys) {
        bool tagged = false;
        for(const QString& t : tags) {
            if(t.contains(qry, Qt::CaseInsensitive)) {
                tagged = true;
            }
        }
        bool prop = false;
        for(const QString& p : properties) {
            if(p.contains(qry, Qt::CaseInsensitive)) {
                prop = true;
            }
        }

        contains_all &= (name.contains(qry, Qt::CaseInsensitive) || descr.contains(qry, Qt::CaseInsensitive) || prop || tagged);
    }
    return contains_all;
}

bool NodeFilterProxyModel::lessThan(const QModelIndex &left, const QModelIndex &right) const
{
    return QSortFilterProxyModel::lessThan(left, right);
}
/// MOC
#include "../../../include/csapex/view/node/moc_node_filter_proxy_model.cpp"
