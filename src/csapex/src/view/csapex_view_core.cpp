/// HEADER
#include <csapex/view/csapex_view_core.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>
#include <csapex/view/utility/message_renderer_manager.h>
#include <csapex/view/node/node_adapter_factory.h>
#include <csapex/view/designer/drag_io.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/command/dispatcher.h>

/// SYSTEM
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/version.hpp>
#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bf3 = boost::filesystem;
#else
namespace bf3 = boost::filesystem3;
#endif

using namespace csapex;

CsApexViewCore::CsApexViewCore()
{
}

DesignerStyleable& CsApexViewCore::getStyle()
{
    return style;
}

bool CsApexViewCore::isDebug() const
{
    return getSettings().getTemporary<bool>("debug", false);
}

bool CsApexViewCore::isGridLockEnabled() const
{
    return getSettings().getPersistent<bool>("grid-lock", false);
}

bool CsApexViewCore::isRemote() const
{
    return false;
}
