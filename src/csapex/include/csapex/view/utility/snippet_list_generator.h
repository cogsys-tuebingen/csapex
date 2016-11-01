#ifndef SNIPPET_LIST_GENERATOR_H
#define SNIPPET_LIST_GENERATOR_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/factory/factory_fwd.h>
#include <csapex/view/node/node_adapter_factory.h>

/// SYSTEM
#include <QMenu>
#include <QTreeWidget>
#include <QStandardItemModel>

namespace csapex
{

class CSAPEX_QT_EXPORT SnippetListGenerator
{
public:
    SnippetListGenerator(SnippetFactory &snippet_factory);

    void insertAvailableSnippets(QMenu* menu);
    void insertAvailableSnippets(QTreeWidget *tree);
    void listAvailableSnippets(QStandardItemModel *model);

private:
    SnippetFactory &snippet_factory_;
};

}

#endif // SNIPPET_LIST_GENERATOR_H
