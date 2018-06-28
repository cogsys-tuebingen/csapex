/// HEADER
#include <csapex/view/csapex_view_core_proxy.h>


/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/command/dispatcher_proxy.h>
#include <csapex/core/bootstrap.h>
#include <csapex/core/csapex_core.h>
#include <csapex/core/core_plugin.h>
#include <csapex/plugin/plugin_manager.hpp>
#include <csapex/info.h>
#include <csapex/factory/node_factory_proxy.h>
#include <csapex/io/broadcast_message.h>
#include <csapex/io/channel.h>
#include <csapex/io/protcol/core_notes.h>
#include <csapex/io/protcol/core_requests.h>
#include <csapex/io/protcol/notification_message.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph_facade_proxy.h>
#include <csapex/model/graph/graph_proxy.h>
#include <csapex/model/node_facade_proxy.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/profiling/profiler_proxy.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/serialization/packet_serializer.h>
#include <csapex/view/designer/drag_io.h>
#include <csapex/view/gui_exception_handler.h>
#include <csapex/view/node/node_adapter_factory.h>
#include <csapex/manager/message_renderer_manager.h>

using namespace csapex;


CsApexViewCoreProxy::CsApexViewCoreProxy(const SessionPtr& session)
    : Proxy(session),
      bootstrap_(std::make_shared<Bootstrap>()),
      thread_active_(false),
      exception_handler_(std::make_shared<GuiExceptionHandler>(false))
{
    session_->start();

    spinner = std::thread([&](){
        thread_active_ = true;

        session_->run();

        thread_active_ = false;

        server_shutdown();
    });

    // busy waiting for the spinning thread
    while(!session_->isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    core_channel_ = session_->openChannel(AUUID::NONE);

    profiler_proxy_ = std::make_shared<ProfilerProxy>(core_channel_);

    observe(core_channel_->note_received, [this](const io::NoteConstPtr& note){
        if(const std::shared_ptr<CoreNote const>& cn = std::dynamic_pointer_cast<CoreNote const>(note)) {
            switch(cn->getNoteType()) {
            case CoreNoteType::ConfigChanged:
                config_changed();
                break;
            case CoreNoteType::StepBegin:
                begin_step();
                break;
            case CoreNoteType::StepEnd:
                end_step();
                break;

            case CoreNoteType::StatusChanged:
                status_changed(cn->getPayload<std::string>(0));
                break;
            case CoreNoteType::NewNodeType:
                new_node_type();
                break;
            case CoreNoteType::NewSnippetType:
                new_snippet_type();
                break;
            case CoreNoteType::ResetRequested:
                reset_requested();
                break;
            case CoreNoteType::ResetDone:
                reset_done();
                break;
            case CoreNoteType::Saved:
                saved();
                break;
            case CoreNoteType::Loaded:
                loaded();
                break;
            case CoreNoteType::Paused:
                paused(cn->getPayload<bool>(0));
                break;
            case CoreNoteType::SteppingEnabled:
                stepping_enabled();
                break;

                // TODO: support this
//            case CoreNoteType::SaveDetailRequest:
//                save_detail_request();
//                break;
//            case CoreNoteType::LoadDetailRequest:
//                load_detail_request();
//                break;

            case CoreNoteType::Notification:
                notification(cn->getPayload<Notification>(0));
                break;
            }
        }
    });


    settings_ = std::make_shared<SettingsProxy>(session_);
    remote_plugin_locator_ = std::make_shared<PluginLocator>(*settings_);

    bootstrap_->bootFrom(csapex::info::CSAPEX_DEFAULT_BOOT_PLUGIN_DIR,
                         remote_plugin_locator_.get());

    // init core plugins
    core_plugin_manager = std::make_shared<PluginManager<csapex::CorePlugin>>("csapex::CorePlugin");

    core_plugin_manager->load(remote_plugin_locator_.get());

    for(const auto& cp : core_plugin_manager->getConstructors()) {
        const std::string& plugin_name = cp.first;
        assert(core_plugins_.find(plugin_name) == core_plugins_.end());

        const PluginConstructor<CorePlugin>& constructor = core_plugin_manager->getConstructor(plugin_name);

        CorePlugin::Ptr plugin = constructor();
        plugin->setName(plugin_name);

        core_plugins_[plugin_name] = plugin;
    }

    for(auto plugin : core_plugins_) {
        plugin.second->prepare(getSettings());
    }

    // make the proxys only _after_ the session is started
    NodeFacadeProxyPtr remote_facade = std::make_shared<NodeFacadeProxy>(session_, AUUID::NONE);
    remote_root_ = std::make_shared<GraphFacadeProxy>(session_, remote_facade);


    node_adapter_factory_ = std::make_shared<NodeAdapterFactory>(*settings_, remote_plugin_locator_.get());
    dispatcher_ = std::make_shared<CommandDispatcherProxy>(session_);

    drag_io = std::make_shared<DragIO>(remote_plugin_locator_, dispatcher_.get());

    // TODO: replace with proxies
    node_factory_ = std::make_shared<NodeFactoryProxy>(session_);
    // thread_pool_ = ...
    // snippet_factory_ = ...

    observe(session_->packet_received, [this](StreamableConstPtr packet){
        handlePacket(packet);
    });
    observe(session_->broadcast_received, [this](BroadcastMessageConstPtr packet){
        handleBroadcast(packet);
    });

    MessageRendererManager::instance().setPluginLocator(remote_plugin_locator_);
    node_adapter_factory_->loadPlugins();


    observe(dispatcher_->state_changed, undo_state_changed);
    observe(dispatcher_->dirty_changed, undo_dirty_changed);
}

CsApexViewCoreProxy::~CsApexViewCoreProxy()
{
    MessageRendererManager::instance().shutdown();

    session_->shutdown();

    if(spinner.joinable()) {
        spinner.join();
    }
    remote_plugin_locator_->shutdown();
}


bool CsApexViewCoreProxy::isProxy() const
{
    return true;
}

void CsApexViewCoreProxy::handlePacket(StreamableConstPtr packet)
{
    (void) packet;
}

void CsApexViewCoreProxy::handleBroadcast(BroadcastMessageConstPtr packet)
{
    if(packet) {
        //                std::cout << "type=" << (int) serial->getPacketType() << std::endl;

        switch(packet->getPacketType()) {
        case BroadcastMessage::PACKET_TYPE_ID:
            if(BroadcastMessageConstPtr broadcast = std::dynamic_pointer_cast<BroadcastMessage const>(packet)) {
                if(auto notification_msg = std::dynamic_pointer_cast<NotificationMessage const>(broadcast)) {
                    Notification n = notification_msg->getNotification();
                    notification(n);
                }
            }
        }
    }
}


NodeAdapterFactoryPtr CsApexViewCoreProxy::getNodeAdapterFactory()
{
    return node_adapter_factory_;
}

std::shared_ptr<DragIO> CsApexViewCoreProxy::getDragIO()
{
    return drag_io;
}

/// PROXIES
ExceptionHandler& CsApexViewCoreProxy::getExceptionHandler() const
{
    return *exception_handler_;
}


PluginLocatorPtr CsApexViewCoreProxy::getPluginLocator() const
{
    return remote_plugin_locator_;
}

CommandExecutorPtr CsApexViewCoreProxy::getCommandDispatcher()
{
    return dispatcher_;
}

Settings& CsApexViewCoreProxy::getSettings() const
{
    return *settings_;
}


GraphFacadePtr CsApexViewCoreProxy::getRoot()
{
    return remote_root_;
}

ThreadPoolPtr CsApexViewCoreProxy::getThreadPool()
{
    return thread_pool_;
}
NodeFactoryPtr CsApexViewCoreProxy::getNodeFactory() const
{
    apex_assert_hard(node_factory_);
    return node_factory_;
}
SnippetFactoryPtr CsApexViewCoreProxy::getSnippetFactory() const
{
    // TODO: support in remote view
    return snippet_factory_;
}
ProfilerPtr CsApexViewCoreProxy::getProfiler() const
{
    return profiler_proxy_;
}

void CsApexViewCoreProxy::sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level)
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSendNotification, notification, static_cast<uint8_t>(error_level));
}



/// RELAYS

void CsApexViewCoreProxy::reset()
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreReset);
}


void CsApexViewCoreProxy::load(const std::string& file)
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreLoad, file);
}

void CsApexViewCoreProxy::saveAs(const std::string& file, bool quiet)
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSave, file, quiet);
}

SnippetPtr CsApexViewCoreProxy::serializeNodes(const AUUID &graph_id, const std::vector<UUID>& nodes) const
{
    return request<SnippetPtr, CoreRequests>(CoreRequests::CoreRequestType::CoreSerialize, graph_id, nodes);
}

bool CsApexViewCoreProxy::isPaused() const
{
    return request<bool, CoreRequests>(CoreRequests::CoreRequestType::CoreGetPause);
}

void CsApexViewCoreProxy::setPause(bool paused)
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSetPause, paused);
}


bool CsApexViewCoreProxy::isSteppingMode() const
{
    return request<bool, CoreRequests>(CoreRequests::CoreRequestType::CoreGetSteppingMode);
}

void CsApexViewCoreProxy::setSteppingMode(bool stepping)
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSetSteppingMode, stepping);
}

void CsApexViewCoreProxy::step()
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreStep);
}


void CsApexViewCoreProxy::shutdown()
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreShutdown);
}

void CsApexViewCoreProxy::clearBlock()
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreClearBlock);
}

void CsApexViewCoreProxy::resetActivity()
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreResetActivity);
}
