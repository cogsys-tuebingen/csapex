/// HEADER
#include <csapex/view/csapex_view_core_local.h>

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

CsApexViewCoreLocal::CsApexViewCoreLocal(CsApexCorePtr core)
    : core_(core),
      node_adapter_factory_(std::make_shared<NodeAdapterFactory>(core_->getSettings(), core_->getPluginLocator().get())),
      dispatcher_(core_->getCommandDispatcher()),
      node_factory_(core_->getNodeFactory()),
      snippet_factory_(core_->getSnippetFactory()),
      drag_io(std::make_shared<DragIO>(core_->getPluginLocator(), dispatcher_.get())),
      exception_handler_(core_->getExceptionHandler())
{
    MessageRendererManager::instance().setPluginLocator(core_->getPluginLocator());
    node_adapter_factory_->loadPlugins();

    observe(core_->config_changed, config_changed);
    observe(core_->status_changed, status_changed);
    observe(core_->new_node_type, new_node_type);
    observe(core_->new_snippet_type, new_snippet_type);
    observe(core_->reset_requested, reset_requested);
    observe(core_->reset_done, reset_done);
    observe(core_->saved, saved);
    observe(core_->loaded, loaded);
    observe(core_->paused, paused);
    observe(core_->begin_step, begin_step);
    observe(core_->end_step, end_step);


    observe(core_->getRoot()->node_added, node_added);
    observe(core_->getRoot()->node_removed, node_removed);
    observe(core_->getRoot()->node_worker_added, node_worker_added);
    observe(core_->getRoot()->node_worker_removed, node_worker_removed);
    observe(core_->getRoot()->panic, panic);


    observe(core_->getThreadPool()->group_created, group_created);
    observe(core_->getThreadPool()->group_removed, group_removed);


    observe(dispatcher_->state_changed, undo_state_changed);
    observe(dispatcher_->dirty_changed, undo_dirty_changed);

    observe(core_->notification, notification);
}

CsApexViewCoreLocal::CsApexViewCoreLocal(CsApexViewCore &parent, ExceptionHandler &exception_handler)
    : core_(new CsApexCore(parent.getSettings(), parent.getExceptionHandler(), parent.getPluginLocator(),
                           parent.getNodeFactory(), parent.getSnippetFactory())),
      dispatcher_(std::make_shared<CommandDispatcher>(*core_)),
      exception_handler_(exception_handler)
{
    node_factory_ = parent.getNodeFactory();
    snippet_factory_ = parent.getSnippetFactory();
    node_adapter_factory_ = parent.getNodeAdapterFactory();
    drag_io = parent.getDragIO();

    core_->init();
}



NodeAdapterFactoryPtr CsApexViewCoreLocal::getNodeAdapterFactory()
{
    return node_adapter_factory_;
}

std::shared_ptr<DragIO> CsApexViewCoreLocal::getDragIO()
{
    return drag_io;
}

/// PROXIES
ExceptionHandler& CsApexViewCoreLocal::getExceptionHandler() const
{
    return exception_handler_;
}


PluginLocatorPtr CsApexViewCoreLocal::getPluginLocator() const
{
    return core_->getPluginLocator();
}

CommandExecutorPtr CsApexViewCoreLocal::getCommandDispatcher()
{
    return dispatcher_;
}

Settings& CsApexViewCoreLocal::getSettings() const
{
    return core_->getSettings();
}


GraphFacadePtr CsApexViewCoreLocal::getRoot()
{
    return core_->getRoot();
}

ThreadPoolPtr CsApexViewCoreLocal::getThreadPool()
{
    // TODO: replace with proxy
    apex_assert_hard(core_->getThreadPool());
    return core_->getThreadPool();
}
NodeFactoryPtr CsApexViewCoreLocal::getNodeFactory() const
{
    // TODO: replace with proxy
    apex_assert_hard(node_factory_);
    return node_factory_;
}
SnippetFactoryPtr CsApexViewCoreLocal::getSnippetFactory() const
{
    // TODO: replace with proxy
    apex_assert_hard(snippet_factory_);
    return snippet_factory_;
}
ProfilerPtr CsApexViewCoreLocal::getProfiler() const
{
    // TODO: replace with proxy
    apex_assert_hard(core_->getProfiler() != nullptr);
    return core_->getProfiler();
}

void CsApexViewCoreLocal::sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level)
{
    core_->sendNotification(notification, error_level);
}



/// RELAYS

void CsApexViewCoreLocal::reset()
{
    core_->reset();
}


void CsApexViewCoreLocal::load(const std::string& file)
{
    core_->load(file);
}

void CsApexViewCoreLocal::saveAs(const std::string& file, bool quiet)
{
    core_->saveAs(file, quiet);
}

bool CsApexViewCoreLocal::isPaused() const
{
    return core_->isPaused();
}

void CsApexViewCoreLocal::setPause(bool paused)
{
    core_->setPause(paused);
}


bool CsApexViewCoreLocal::isSteppingMode() const
{
    return core_->isSteppingMode();
}

void CsApexViewCoreLocal::setSteppingMode(bool stepping)
{
    core_->setSteppingMode(stepping);
}

void CsApexViewCoreLocal::step()
{
    core_->step();
}


void CsApexViewCoreLocal::shutdown()
{
    core_->shutdown();
}

void CsApexViewCoreLocal::clearBlock()
{
    core_->getRoot()->clearBlock();
}

void CsApexViewCoreLocal::resetActivity()
{
    core_->getRoot()->resetActivity();
}
