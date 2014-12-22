/// HEADER
#include <csapex/core/csapex_core.h>

/// COMPONENT
#include <csapex/model/node_factory.h>
#include <csapex/core/core_plugin.h>
#include <csapex/core/graphio.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/command/dispatcher.h>
#include <csapex/msg/message.h>
#include <csapex/msg/message_factory.h>
#include <csapex/utility/plugin_manager.hpp>
#include <csapex/model/tag.h>
#include <csapex/utility/assert.h>
#include <csapex/model/graph_worker.h>
#include <csapex/utility/register_msg.h>
#include <csapex/view/node_adapter_factory.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/core/bootstrap_plugin.h>
#include <csapex/manager/message_provider_manager.h>
#include <csapex/manager/message_renderer_manager.h>

/// SYSTEM
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

Q_DECLARE_METATYPE(QSharedPointer<QImage>)
Q_DECLARE_METATYPE(std::string)

using namespace csapex;

CsApexCore::CsApexCore(Settings &settings, PluginLocatorPtr plugin_locator,
                       GraphWorkerPtr graph, NodeFactory *node_factory, NodeAdapterFactory *node_adapter_factory, CommandDispatcher* cmd_dispatcher)
    : settings_(settings), plugin_locator_(plugin_locator), graph_worker_(graph),
      node_factory_(node_factory), node_adapter_factory_(node_adapter_factory),
      cmd_dispatch(cmd_dispatcher), core_plugin_manager(new PluginManager<csapex::CorePlugin>("csapex::CorePlugin")), init_(false)
{
    destruct = true;

    qRegisterMetaType < QSharedPointer<QImage> > ("QSharedPointer<QImage>");
    qRegisterMetaType < ConnectionType::Ptr > ("ConnectionType::Ptr");
    qRegisterMetaType < std::string > ("std::string");

    StreamInterceptor::instance().start();

    settings.settingsChanged.connect(boost::bind(&CsApexCore::settingsChanged, this));

    QObject::connect(graph_worker_.get(), SIGNAL(paused(bool)), this, SIGNAL(paused(bool)));
}

CsApexCore::~CsApexCore()
{
    StreamInterceptor::instance().stop();

    MessageProviderManager::instance().shutdown();
    MessageRendererManager::instance().shutdown();

    for(std::vector<CorePlugin::Ptr>::iterator it = core_plugins_.begin(); it != core_plugins_.end(); ++it){
        (*it)->shutdown();
    }
    core_plugins_.clear();
    if(destruct) {
        delete core_plugin_manager;
    }

    boot_plugins_.clear();
    while(!boot_plugin_loaders_.empty()) {
        delete boot_plugin_loaders_.front();
        boot_plugin_loaders_.erase(boot_plugin_loaders_.begin());
    }

}

void CsApexCore::setPause(bool pause)
{
    if(pause != graph_worker_->isPaused()) {
        std::cout << (pause ? "pause" : "unpause") << std::endl;
        graph_worker_->setPause(pause);
    }
}

bool CsApexCore::isPaused() const
{
    return graph_worker_->isPaused();
}

void CsApexCore::setStatusMessage(const std::string &msg)
{
    Q_EMIT showStatusMessage(msg);
}

void CsApexCore::init(DragIO* dragio)
{
    if(!init_) {
        init_ = true;

        showStatusMessage("booting up");
        boot();

        showStatusMessage("loading core plugins");
        core_plugin_manager->load(plugin_locator_.get());

        typedef const std::pair<std::string, PluginManager<CorePlugin>::Constructor> PAIR;
        Q_FOREACH(PAIR cp, core_plugin_manager->availableClasses()) {
            CorePlugin::Ptr plugin = cp.second();
            core_plugins_.push_back(plugin);
        }
        Q_FOREACH(CorePlugin::Ptr plugin, core_plugins_) {
            plugin->prepare(getSettings());
        }
        Q_FOREACH(CorePlugin::Ptr plugin, core_plugins_) {
            plugin->init(*this);
        }
        if(dragio) {
            Q_FOREACH(CorePlugin::Ptr plugin, core_plugins_) {
                plugin->initUI(*dragio);
            }
        }

        showStatusMessage("loading node plugins");
        node_factory_->loaded.connect(boost::bind(&CsApexCore::showStatusMessage, this, _1));
        node_factory_->new_node_type.connect(boost::bind(&CsApexCore::reloadBoxMenues, this));
        node_factory_->loadPlugins();

        node_adapter_factory_->loadPlugins();
    }
}

void CsApexCore::boot()
{
    std::string dir_string = CSAPEX_BOOT_PLUGIN_DIR;

    boost::filesystem::path directory(dir_string);
    boost::filesystem::directory_iterator dir(directory);
    boost::filesystem::directory_iterator end;

    for(; dir != end; ++dir) {
        boost::filesystem::path path = dir->path();

        boot_plugin_loaders_.push_back(new class_loader::ClassLoader(path.string()));
        class_loader::ClassLoader* loader = boot_plugin_loaders_.back();

        try {
            std::vector<std::string> classes = loader->getAvailableClasses<BootstrapPlugin>();

            for(std::size_t c = 0; c < classes.size(); ++c){
                boost::shared_ptr<BootstrapPlugin> plugin = loader->createInstance<BootstrapPlugin>(classes[c]);
                boot_plugins_.push_back(plugin);

                plugin->boot(plugin_locator_.get());
            }
        } catch(const std::exception& e) {
            std::cerr << "boot plugin " << path << " failed: " << e.what() << std::endl;
        }
    }

    MessageProviderManager::instance().setPluginLocator(plugin_locator_);
    MessageRendererManager::instance().setPluginLocator(plugin_locator_);
}

void CsApexCore::startup()
{
    showStatusMessage("loading config");
    try {
        load(settings_.get<std::string>("config", Settings::defaultConfigFile()));
    } catch(const std::exception& e) {
        std::cerr << "error loading the config: " << e.what() << std::endl;
    }
    showStatusMessage("painting user interface");
}

void CsApexCore::reset()
{
    Q_EMIT resetRequest();

    cmd_dispatch->reset();

    UUID::reset();

    Q_FOREACH(Listener* l, listener_) {
        l->resetSignal();
    }
}

void CsApexCore::addListener(Listener *l)
{
    listener_.push_back(l);
}

void CsApexCore::removeListener(Listener *l)
{
    std::vector<Listener*>::iterator it = std::find(listener_.begin(), listener_.end(), l);
    if(it != listener_.end()) {
        listener_.erase(it);
    }
}


Settings &CsApexCore::getSettings() const
{
    return settings_;
}

NodeFactory &CsApexCore::getNodeFactory() const
{
    return *node_factory_;
}

void CsApexCore::settingsChanged()
{
    settings_.save();
    Q_EMIT configChanged();
}

void CsApexCore::saveAs(const std::string &file)
{
    std::string dir = file.substr(0, file.find_last_of('/')+1);

    if(!dir.empty()) {
        int chdir_result = chdir(dir.c_str());
        if(chdir_result != 0) {
            throw std::runtime_error(std::string("cannot change into directory ") + dir);
        }
    }


    YAML::Node node_map(YAML::NodeType::Map);

    GraphIO graphio(graph_worker_->getGraph(), node_factory_);

    Q_EMIT saveSettingsRequest(node_map);

    graphio.saveSettings(node_map);
    graphio.saveConnections(node_map);

    Q_EMIT saveViewRequest(node_map);

    graphio.saveNodes(node_map);

    YAML::Emitter yaml;
    yaml << node_map;

    std::ofstream ofs(file.c_str());
    ofs << "#!" << settings_.get<std::string>("path_to_bin") << '\n';
    ofs << yaml.c_str();

    std::cout << "save: " << yaml.c_str() << std::endl;

    cmd_dispatch->setClean();
    cmd_dispatch->resetDirtyPoint();
}


void CsApexCore::load(const std::string &file)
{
    settings_.set("config", file);

    reset();

    apex_assert_hard(graph_worker_->getGraph()->countNodes() == 0);

    graph_worker_->setPause(true);

    GraphIO graphio(graph_worker_->getGraph(), node_factory_);

    {
        std::ifstream ifs(file.c_str());
        YAML::Parser parser(ifs);

        YAML::NodeBuilder builder;
        if (!parser.HandleNextDocument(builder)) {
            std::cerr << "cannot read the config" << std::endl;
        }
        YAML::Node doc = builder.Root();

        graphio.loadSettings(doc);

        YAML::Node nodes = doc["nodes"];
        if(nodes.IsDefined()) {
            for(std::size_t i = 0, total = nodes.size(); i < total; ++i) {
                graphio.loadNode(doc["nodes"][i]);
            }
        }

        // legacy nodes
        while (parser.HandleNextDocument(builder)) {
            YAML::Node doc = builder.Root();
            graphio.loadNode(doc);
        }
    }
    {
        std::ifstream ifs(file.c_str());
        YAML::Parser parser(ifs);

        YAML::NodeBuilder builder;
        if (!parser.HandleNextDocument(builder)) {
            std::cerr << "cannot read the config" << std::endl;
        }
        YAML::Node doc = builder.Root();

        graphio.loadConnections(doc);

        Q_EMIT loadViewRequest(doc);

        Q_EMIT loadSettingsRequest(doc);
    }

    cmd_dispatch->setClean();
    cmd_dispatch->resetDirtyPoint();

    graph_worker_->setPause(false);
}
