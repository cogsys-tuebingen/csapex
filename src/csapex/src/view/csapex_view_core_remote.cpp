/// HEADER
#include <csapex/view/csapex_view_core_remote.h>


/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/command/dispatcher_remote.h>
#include <csapex/core/csapex_core.h>
#include <csapex/io/broadcast_message.h>
#include <csapex/io/channel.h>
#include <csapex/io/protcol/core_requests.h>
#include <csapex/io/protcol/core_notes.h>
#include <csapex/io/protcol/notification_message.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/graph_facade_remote.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/graph/graph_remote.h>
#include <csapex/model/node_facade_remote.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/serialization/packet_serializer.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/profiling/profiler_remote.h>
#include <csapex/view/designer/drag_io.h>
#include <csapex/view/gui_exception_handler.h>
#include <csapex/view/node/node_adapter_factory.h>
#include <csapex/view/utility/message_renderer_manager.h>

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


CsApexViewCoreRemote::CsApexViewCoreRemote(Session& session, CsApexCorePtr core_tmp)
    : Remote(session),
      exception_handler_(std::make_shared<GuiExceptionHandler>(false)),
      core_tmp_(core_tmp)
{
    session_.start();

    running = true;
    spinner = std::thread([&](){
        while(running) {
            session_.run();
        }
    });

    core_channel_ = session_.openChannel(AUUID::NONE);

    profiler_proxy_ = std::make_shared<ProfilerRemote>(core_channel_);

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

    // make the proxys only _after_ the session is started
    NodeFacadeRemotePtr remote_facade = std::make_shared<NodeFacadeRemote>(session_, AUUID::NONE);
    remote_root_ = std::make_shared<GraphFacadeRemote>(session_, remote_facade);
    settings_ = std::make_shared<SettingsRemote>(session_);
    remote_plugin_locator_ = std::make_shared<PluginLocator>(core_tmp->getSettings());//(*settings_);
    //boot remote_plugin_locator_

    node_adapter_factory_ = std::make_shared<NodeAdapterFactory>(*settings_, remote_plugin_locator_.get());
    dispatcher_ = std::make_shared<CommandDispatcherRemote>(session_);

    drag_io = std::make_shared<DragIO>(core_tmp_->getPluginLocator(), dispatcher_.get());

    //    dispatcher_ = core_tmp_->getCommandDispatcher();
    node_factory_ = core_tmp_->getNodeFactory();
    snippet_factory_ = core_tmp_->getSnippetFactory();

    observe(session_.packet_received, [this](StreamableConstPtr packet){
        handlePacket(packet);
    });
    observe(session_.broadcast_received, [this](BroadcastMessageConstPtr packet){
        handleBroadcast(packet);
    });

    MessageRendererManager::instance().setPluginLocator(getPluginLocator());
    node_adapter_factory_->loadPlugins();


    observe(dispatcher_->state_changed, undo_state_changed);
    observe(dispatcher_->dirty_changed, undo_dirty_changed);
}

CsApexViewCoreRemote::~CsApexViewCoreRemote()
{
    if(spinner.joinable()) {
        spinner.join();
    }
    remote_plugin_locator_->shutdown();
}


void CsApexViewCoreRemote::handlePacket(StreamableConstPtr packet)
{
}

void CsApexViewCoreRemote::handleBroadcast(BroadcastMessageConstPtr packet)
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


NodeAdapterFactoryPtr CsApexViewCoreRemote::getNodeAdapterFactory()
{
    return node_adapter_factory_;
}

std::shared_ptr<DragIO> CsApexViewCoreRemote::getDragIO()
{
    return drag_io;
}

/// PROXIES
ExceptionHandler& CsApexViewCoreRemote::getExceptionHandler() const
{
    return *exception_handler_;
}


PluginLocatorPtr CsApexViewCoreRemote::getPluginLocator() const
{
    return core_tmp_->getPluginLocator();
}

CommandExecutorPtr CsApexViewCoreRemote::getCommandDispatcher()
{
    return dispatcher_;
}

Settings& CsApexViewCoreRemote::getSettings() const
{
    return *settings_;
}


GraphFacadePtr CsApexViewCoreRemote::getRoot()
{
    return remote_root_;
}

ThreadPoolPtr CsApexViewCoreRemote::getThreadPool()
{
    // TODO: replace with proxy
    //apex_assert_hard(//core_->getThreadPool());
    return core_tmp_->getThreadPool();
}
NodeFactoryPtr CsApexViewCoreRemote::getNodeFactory() const
{
    // TODO: replace with proxy
    apex_assert_hard(node_factory_);
    return node_factory_;
}
SnippetFactoryPtr CsApexViewCoreRemote::getSnippetFactory() const
{
    // TODO: replace with proxy
    apex_assert_hard(snippet_factory_);
    return snippet_factory_;
}
ProfilerPtr CsApexViewCoreRemote::getProfiler() const
{
    return profiler_proxy_;
}

void CsApexViewCoreRemote::sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level)
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSendNotification, notification, static_cast<uint8_t>(error_level));
}



/// RELAYS

void CsApexViewCoreRemote::reset()
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreReset);
}


void CsApexViewCoreRemote::load(const std::string& file)
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreLoad, file);
}

void CsApexViewCoreRemote::saveAs(const std::string& file, bool quiet)
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSave, file, quiet);
}

SnippetPtr CsApexViewCoreRemote::serializeNodes(const AUUID &graph_id, const std::vector<UUID>& nodes) const
{
    return request<SnippetPtr, CoreRequests>(CoreRequests::CoreRequestType::CoreSerialize, graph_id, nodes);
}

bool CsApexViewCoreRemote::isPaused() const
{
    return request<bool, CoreRequests>(CoreRequests::CoreRequestType::CoreGetPause);
}

void CsApexViewCoreRemote::setPause(bool paused)
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSetPause, paused);
}


bool CsApexViewCoreRemote::isSteppingMode() const
{
    return request<bool, CoreRequests>(CoreRequests::CoreRequestType::CoreGetSteppingMode);
}

void CsApexViewCoreRemote::setSteppingMode(bool stepping)
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSetSteppingMode, stepping);
}

void CsApexViewCoreRemote::step()
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreStep);
}


void CsApexViewCoreRemote::shutdown()
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreShutdown);
    running = false;
    session_.shutdown();
}

void CsApexViewCoreRemote::clearBlock()
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreClearBlock);
}

void CsApexViewCoreRemote::resetActivity()
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreResetActivity);
}
