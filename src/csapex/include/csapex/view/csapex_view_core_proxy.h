#ifndef CSAPEX_VIEW_CORE_PROXY_H
#define CSAPEX_VIEW_CORE_PROXY_H

/// COMPONENT
#include <csapex/view/csapex_view_core.h>
#include <csapex/core/settings/settings_proxy.h>
#include <csapex/io/proxy.h>
#include <csapex/core/core_fwd.h>

/// SYSTEM
#include <thread>

namespace csapex
{
class CsApexCore;
class NodeAdapterFactory;
class CommandDispatcherProxy;

class DesignerStyleable;
class DesignerOptions;
class DragIO;

class ProfilerProxy;

class CSAPEX_QT_EXPORT CsApexViewCoreProxy : public CsApexViewCore, public Proxy
{
public:
    CsApexViewCoreProxy(const SessionPtr &session);
    ~CsApexViewCoreProxy();

    bool isProxy() const override;

    void sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level = ErrorState::ErrorLevel::ERROR) override;

    // CORE
    void reset() override;
    void load(const std::string& file) override;
    void saveAs(const std::string& file, bool quiet = false) override;

    SnippetPtr serializeNodes(const AUUID& graph_id, const std::vector<UUID>& nodes) const override;

    void setPause(bool paused) override;
    bool isPaused() const override;

    bool isSteppingMode() const override;
    void setSteppingMode(bool stepping) override;
    void step() override;

    Settings& getSettings() const override;    
    CommandExecutorPtr getCommandDispatcher() override;
    ExceptionHandler& getExceptionHandler() const override;

    void shutdown() override;
    void clearBlock() override;
    void resetActivity() override;

    GraphFacadePtr getRoot() override;

    ProfilerPtr getProfiler() const override;

    std::shared_ptr<DragIO> getDragIO() override;
    std::shared_ptr<NodeAdapterFactory> getNodeAdapterFactory() override;

    PluginLocatorPtr getPluginLocator() const override;

    // TODO: add proxies or remove
    NodeFactoryPtr getNodeFactory() const override;

    /**
     * @brief getThreadPool
     * @return nullptr, iff no thread pool exists
     */
    ThreadPoolPtr getThreadPool() override;

    /**
     * @brief getSnippetFactory
     * @return nullptr, iff no factory exists
     */
    SnippetFactoryPtr getSnippetFactory() const override;

private:
    void handlePacket(StreamableConstPtr packet);
    void handleBroadcast(BroadcastMessageConstPtr packet);

private:
    BootstrapPtr bootstrap_;

    io::ChannelPtr core_channel_;

    PluginLocatorPtr remote_plugin_locator_;

    std::shared_ptr<SettingsProxy> settings_;

    std::shared_ptr<NodeAdapterFactory> node_adapter_factory_;
    std::shared_ptr<CommandDispatcherProxy> dispatcher_;

    std::shared_ptr<ThreadPool> thread_pool_;
    std::shared_ptr<NodeFactory> node_factory_;
    std::shared_ptr<SnippetFactory> snippet_factory_;

    std::shared_ptr<DragIO> drag_io;

    std::shared_ptr<ProfilerProxy> profiler_proxy_;

    bool thread_active_;
    std::thread spinner;

    std::shared_ptr<ExceptionHandler> exception_handler_;

    GraphFacadePtr remote_root_;
};

}

#endif // CSAPEX_VIEW_CORE_PROXY_H
