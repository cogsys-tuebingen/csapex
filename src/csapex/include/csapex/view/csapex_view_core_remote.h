#ifndef CSAPEX_VIEW_CORE_REMOTE_H
#define CSAPEX_VIEW_CORE_REMOTE_H

/// COMPONENT
#include <csapex/view/csapex_view_core.h>
#include <csapex/core/settings/settings_remote.h>
#include <boost/asio.hpp>
#include <thread>

namespace csapex
{
class CsApexCore;
class NodeAdapterFactory;
class CommandDispatcherRemote;

class DesignerStyleable;
class DesignerOptions;
class DragIO;

class CSAPEX_QT_EXPORT CsApexViewCoreRemote : public CsApexViewCore
{
public:
    CsApexViewCoreRemote(const std::string& ip, int port, CsApexCorePtr core_tmp);
    ~CsApexViewCoreRemote();

    void sendNotification(const std::string& notification, ErrorState::ErrorLevel error_level = ErrorState::ErrorLevel::ERROR) override;

    // CORE
    void reset() override;
    void load(const std::string& file) override;
    void saveAs(const std::string& file, bool quiet = false) override;

    void setPause(bool paused) override;
    bool isPaused() const override;

    bool isSteppingMode() const override;
    void setSteppingMode(bool stepping) override;
    void step() override;

    Settings& getSettings() const override;

    // TODO: add proxies or remove
    ExceptionHandler& getExceptionHandler() const override;
    GraphFacadePtr getRoot() override;
    void shutdown() override;
    void clearBlock() override;
    void resetActivity() override;

    ThreadPoolPtr getThreadPool() override;

    CommandExecutorPtr getCommandDispatcher() override;

    PluginLocatorPtr getPluginLocator() const override;
    NodeFactoryPtr getNodeFactory() const override;
    SnippetFactoryPtr getSnippetFactory() const override;

    ProfilerPtr getProfiler() const override;

    std::shared_ptr<DragIO> getDragIO() override;
    std::shared_ptr<NodeAdapterFactory> getNodeAdapterFactory() override;

private:
    void handlePacket(SerializableConstPtr packet);
    void handleBroadcast(BroadcastMessageConstPtr packet);

private:

    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;
    boost::asio::ip::tcp::resolver resolver;
    boost::asio::ip::tcp::resolver::iterator resolver_iterator;

    SessionPtr session_;

    std::shared_ptr<SettingsRemote> settings_;

    std::shared_ptr<NodeAdapterFactory> node_adapter_factory_;
    std::shared_ptr<CommandDispatcherRemote> dispatcher_;

    std::shared_ptr<NodeFactory> node_factory_;
    std::shared_ptr<SnippetFactory> snippet_factory_;

    std::shared_ptr<DragIO> drag_io;


    bool running;
    std::thread spinner;

//    ExceptionHandler &exception_handler_;

    // TODO: remove
    CsApexCorePtr core_tmp_;
};

}

#endif // CSAPEX_VIEW_CORE_REMOTE_H
