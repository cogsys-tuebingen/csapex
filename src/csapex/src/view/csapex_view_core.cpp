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

CsApexViewCore::CsApexViewCore(CsApexCorePtr core)
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


    observe(core_->getCommandDispatcher()->state_changed, undo_state_changed);
    observe(core_->getCommandDispatcher()->dirty_changed, undo_dirty_changed);

    observe(core_->notification, notification);
}

CsApexViewCore::CsApexViewCore(CsApexViewCore& parent, ExceptionHandler &exception_handler)
    : core_(new CsApexCore(parent.getSettings(), parent.getExceptionHandler(), parent.getPluginLocator(),
                           parent.getNodeFactory(), parent.getSnippetFactory())),
      dispatcher_(std::make_shared<CommandDispatcher>(*core_)),
      exception_handler_(exception_handler)
{
    node_factory_ = parent.node_factory_;
    snippet_factory_ = parent.snippet_factory_;
    node_adapter_factory_ = parent.node_adapter_factory_;
    drag_io = parent.drag_io;

    core_->init();
}

void CsApexViewCore::execute(const CommandPtr &command)
{
    dispatcher_->execute(command);
}
void CsApexViewCore::executeLater(const CommandPtr &command)
{
    dispatcher_->executeLater(command);
}

void CsApexViewCore::executeLater()
{
    dispatcher_->executeLater();
}

void CsApexViewCore::undo()
{
    dispatcher_->undo();
}
void CsApexViewCore::redo()
{
    dispatcher_->redo();
}


bool CsApexViewCore::canUndo() const
{
    return dispatcher_->canUndo();
}
bool CsApexViewCore::canRedo() const
{
    return dispatcher_->canRedo();
}

bool CsApexViewCore::isDirty() const
{
    return dispatcher_->isDirty();
}

PluginLocatorPtr CsApexViewCore::getPluginLocator() const
{
    return core_->getPluginLocator();
}


NodeAdapterFactoryPtr CsApexViewCore::getNodeAdapterFactory()
{
    return node_adapter_factory_;
}

DesignerStyleable& CsApexViewCore::getStyle()
{
    return style;
}

std::shared_ptr<DragIO> CsApexViewCore::getDragIO()
{
    return drag_io;
}



ExceptionHandler& CsApexViewCore::getExceptionHandler() const
{
    return exception_handler_;
}


/// PROXIES
CommandExecutorPtr CsApexViewCore::getCommandDispatcher()
{
    return dispatcher_;
}

Settings& CsApexViewCore::getSettings()
{
    return core_->getSettings();
}


GraphFacadePtr CsApexViewCore::getRoot()
{
    return core_->getRoot();
}

ThreadPoolPtr CsApexViewCore::getThreadPool()
{
    // TODO: replace with proxy
    apex_assert_hard(core_->getThreadPool());
    return core_->getThreadPool();
}
NodeFactoryPtr CsApexViewCore::getNodeFactory() const
{
    // TODO: replace with proxy
    apex_assert_hard(node_factory_);
    return node_factory_;
}
SnippetFactoryPtr CsApexViewCore::getSnippetFactory() const
{
    // TODO: replace with proxy
    apex_assert_hard(snippet_factory_);
    return snippet_factory_;
}
ProfilerPtr CsApexViewCore::getProfiler() const
{
    // TODO: replace with proxy
    apex_assert_hard(core_->getProfiler() != nullptr);
    return core_->getProfiler();
}

void CsApexViewCore::sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level)
{
    core_->sendNotification(notification, error_level);
}



/// RELAYS

void CsApexViewCore::reset()
{
    core_->reset();
}


void CsApexViewCore::load(const std::string& file)
{
    core_->load(file);
}

void CsApexViewCore::saveAs(const std::string& file, bool quiet)
{
    core_->saveAs(file, quiet);
}

bool CsApexViewCore::isDebug() const
{
    return core_->getSettings().get<bool>("debug", false);
}

bool CsApexViewCore::isGridLockEnabled() const
{
    return core_->getSettings().get<bool>("grid-lock", false);
}

bool CsApexViewCore::isPaused() const
{
    return core_->isPaused();
}

void CsApexViewCore::setPause(bool paused)
{
    core_->setPause(paused);
}


bool CsApexViewCore::isSteppingMode() const
{
    return core_->isSteppingMode();
}

void CsApexViewCore::setSteppingMode(bool stepping)
{
    core_->setSteppingMode(stepping);
}

void CsApexViewCore::step()
{
    core_->step();
}


void CsApexViewCore::shutdown()
{
    core_->shutdown();
}

void CsApexViewCore::clearBlock()
{
    core_->getRoot()->clearBlock();
}

void CsApexViewCore::resetActivity()
{
    core_->getRoot()->resetActivity();
}
