/// HEADER
#include <csapex/view/csapex_view_core.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>
#include <csapex/view/utility/message_renderer_manager.h>
#include <csapex/view/node/node_adapter_factory.h>
#include <csapex/view/designer/drag_io.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>

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

CsApexViewCore::CsApexViewCore(CsApexCore& core)
    : core_(core),
      node_adapter_factory_(std::make_shared<NodeAdapterFactory>(core_.getSettings(), core.getPluginLocator().get())),
      dispatcher_(core_.getCommandDispatcher()),
      drag_io(std::make_shared<DragIO>(core_.getPluginLocator(), dispatcher_.get()))
{
    MessageRendererManager::instance().setPluginLocator(core_.getPluginLocator());
    node_adapter_factory_->loadPlugins();

    observe(core_.saved, [this](){
        dispatcher_->setClean();
        dispatcher_->resetDirtyPoint();

        bool recovery = core_.getSettings().get<bool>("config_recovery", false);
        if(recovery) {
            // delete recovery file
            bf3::path recov_file = core_.getSettings().get<std::string>("config_recovery_file");
            bf3::path current_config  = core_.getSettings().get<std::string>("config");
            if(recov_file != current_config) {
                bf3::remove(recov_file);
                core_.getSettings().set("config_recovery", false);
            }
        }
    });
    observe(core_.loaded, [this](){
        dispatcher_->setClean();
        dispatcher_->resetDirtyPoint();
    });
    observe(core_.reset_requested, [this](){
        dispatcher_->reset();
        core_.getSettings().set("config_recovery", false);
    });

    observe(core_.config_changed, config_changed);
    observe(core_.status_changed, status_changed);
    observe(core_.new_node_type, new_node_type);
    observe(core_.new_snippet_type, new_snippet_type);
    observe(core_.reset_requested, reset_requested);
    observe(core_.reset_done, reset_done);
    observe(core_.saved, saved);
    observe(core_.loaded, loaded);
    observe(core_.paused, paused);
    observe(core_.begin_step, begin_step);
    observe(core_.end_step, end_step);


    observe(core_.getRoot()->node_added, node_added);
    observe(core_.getRoot()->node_removed, node_removed);
    observe(core_.getRoot()->node_worker_added, node_worker_added);
    observe(core_.getRoot()->node_worker_removed, node_worker_removed);
    observe(core_.getRoot()->panic, panic);


    observe(core_.getThreadPool()->group_created, group_created);
    observe(core_.getThreadPool()->group_removed, group_removed);


    observe(core_.notification, notification);
}

CsApexViewCore::CsApexViewCore(CsApexViewCore& parent, CsApexCore& core, std::shared_ptr<CommandDispatcher> dispatcher)
    : core_(core), node_adapter_factory_(parent.node_adapter_factory_), dispatcher_(dispatcher), drag_io(parent.drag_io)
{

}

void CsApexViewCore::execute(const CommandPtr &command)
{
    dispatcher_->execute(command);
}
void CsApexViewCore::executeLater(const CommandPtr &command)
{
    dispatcher_->executeLater(command);
}

CsApexCore& CsApexViewCore::getCore()
{
    return core_;
}

NodeAdapterFactoryPtr CsApexViewCore::getNodeAdapterFactory()
{
    return node_adapter_factory_;
}

CommandDispatcher& CsApexViewCore::getCommandDispatcher()
{
    return *dispatcher_;
}

DesignerStyleable& CsApexViewCore::getStyle()
{
    return style;
}

std::shared_ptr<DragIO> CsApexViewCore::getDragIO()
{
    return drag_io;
}

Settings& CsApexViewCore::getSettings()
{
    // TODO: don't relay remote stettings.
    return core_.getSettings();
}



/// RELAYS

bool CsApexViewCore::isDebug() const
{
    return core_.getSettings().get<bool>("debug", false);
}

bool CsApexViewCore::isGridLockEnabled() const
{
    return core_.getSettings().get<bool>("grid-lock", false);
}

bool CsApexViewCore::isPaused() const
{
    return core_.isPaused();
}

void CsApexViewCore::setPause(bool paused)
{
    core_.setPause(paused);
}


bool CsApexViewCore::isSteppingMode() const
{
    return core_.isSteppingMode();
}

void CsApexViewCore::setSteppingMode(bool stepping)
{
    core_.setSteppingMode(stepping);
}

void CsApexViewCore::step()
{
    core_.step();
}


void CsApexViewCore::stop()
{
    core_.getRoot()->stop();
}

void CsApexViewCore::clearBlock()
{
    core_.getRoot()->clearBlock();
}

void CsApexViewCore::resetActivity()
{
    core_.getRoot()->resetActivity();
}
