/// HEADER
#include <csapex/model/graph_facade.h>

using namespace csapex;

GraphFacade::GraphFacade()
{
}

GraphFacade::~GraphFacade()
{
    stopObserving();
}
