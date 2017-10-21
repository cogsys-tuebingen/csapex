#ifndef CSAPEX_VIEW_CORE_REMOTE_H
#define CSAPEX_VIEW_CORE_REMOTE_H

/// COMPONENT
#include <csapex/view/csapex_view_core.h>
#include <csapex/core/settings/settings_remote.h>
#include <csapex/io/remote.h>

/// SYSTEM
#include <thread>

namespace csapex
{
class CsApexCore;
class NodeAdapterFactory;
class CommandDispatcherRemote;

class DesignerStyleable;
class DesignerOptions;
class DragIO;

class CSAPEX_QT_EXPORT CsApexViewCoreRemote : public CsApexViewCore, public Remote
{
public:
    CsApexViewCoreRemote(Session &session, CsApexCorePtr core_tmp);
    ~CsApexViewCoreRemote();

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

    // TODO: add proxies or remove
    GraphFacadePtr getRoot() override;

    ThreadPoolPtr getThreadPool() override;

    PluginLocatorPtr getPluginLocator() const override;
    NodeFactoryPtr getNodeFactory() const override;
    SnippetFactoryPtr getSnippetFactory() const override;

    ProfilerPtr getProfiler() const override;

    std::shared_ptr<DragIO> getDragIO() override;
    std::shared_ptr<NodeAdapterFactory> getNodeAdapterFactory() override;

private:
    void handlePacket(StreamableConstPtr packet);
    void handleBroadcast(BroadcastMessageConstPtr packet);

private:
    std::shared_ptr<SettingsRemote> settings_;

    std::shared_ptr<NodeAdapterFactory> node_adapter_factory_;
    std::shared_ptr<CommandDispatcherRemote> dispatcher_;

    std::shared_ptr<NodeFactory> node_factory_;
    std::shared_ptr<SnippetFactory> snippet_factory_;

    std::shared_ptr<DragIO> drag_io;


    bool running;
    std::thread spinner;

    std::shared_ptr<ExceptionHandler> exception_handler_;

    GraphFacadePtr remote_root_;

    // TODO: remove
    CsApexCorePtr core_tmp_;
};

}

#endif // CSAPEX_VIEW_CORE_REMOTE_H
