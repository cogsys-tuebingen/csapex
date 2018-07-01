#ifndef NODE_COMPLETER_H
#define NODE_COMPLETER_H

/// COMPONENT
#include <csapex_qt_export.h>
#include <csapex/model/tag.h>

/// SYSTEM
#include <QCompleter>
#include <vector>
#include <QSortFilterProxyModel>

namespace csapex
{
class CSAPEX_QT_EXPORT NodeFilterProxyModel : public QSortFilterProxyModel
{
    Q_OBJECT

public:
    NodeFilterProxyModel(QObject* parent = 0);

protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex& sourceParent) const;

    bool lessThan(const QModelIndex& left, const QModelIndex& right) const;
};

}  // namespace csapex

#endif  // NODE_COMPLETER_H
