/// HEADER
#include <csapex/core/csapex_core.h>

/// COMPONENT
#include <csapex/core/bootstrap_plugin.h>
#include <csapex/core/core_plugin.h>
#include <csapex/core/graphio.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/info.h>
#include <csapex/model/graph_facade.h>
#include <csapex/factory/node_factory.h>
#include <csapex/factory/snippet_factory.h>
#include <csapex/model/tag.h>
#include <csapex/factory/message_factory.h>
#include <csapex/msg/message.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/plugin/plugin_manager.hpp>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>
#include <csapex/utility/shared_ptr_tools.hpp>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/core/exception_handler.h>
#include <csapex/model/node_runner.h>
#include <csapex/msg/any_message.h>
#include <csapex/utility/error_handling.h>
#include <csapex/profiling/profiler.h>
#include <csapex/manager/message_provider_manager.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/serialization/snippet.h>

/// SYSTEM
#include <fstream>
#ifdef WIN32
#include <direct.h>
#endif
#include <boost/filesystem.hpp>

using namespace csapex;

CsApexCore::CsApexCore(Settings &settings, ExceptionHandler& handler, csapex::PluginLocatorPtr plugin_locator)
    : parent_(nullptr),
      settings_(settings),
      plugin_locator_(plugin_locator),
      exception_handler_(handler),
      node_factory_(nullptr),
      root_uuid_provider_(std::make_shared<UUIDProvider>()),
      profiler_(std::make_shared<Profiler>()),
      core_plugin_manager(nullptr),
      init_(false), load_needs_reset_(false)
{
    thread_pool_ = std::make_shared<ThreadPool>(exception_handler_, !settings_.get<bool>("threadless"), settings_.get<bool>("thread_grouping"));
    thread_pool_->setPause(settings_.get<bool>("initially_paused"));

    signal_connections_.emplace_back(thread_pool_->paused.connect(paused));

    signal_connections_.emplace_back(thread_pool_->begin_step.connect(begin_step));
    signal_connections_.emplace_back(thread_pool_->end_step.connect(end_step));
}

CsApexCore::CsApexCore(Settings &settings, ExceptionHandler& handler)
    : CsApexCore(settings, handler, std::make_shared<PluginLocator>(settings))
{
    signal_connections_.emplace_back(settings.settingsChanged.connect(std::bind(&CsApexCore::settingsChanged, this)));

    exception_handler_.setCore(this);

    settings_.saveRequest.connect([this](YAML::Node& n){ thread_pool_->saveSettings(n); });
    settings_.loadRequest.connect([this](YAML::Node& n){ thread_pool_->loadSettings(n); });

    StreamInterceptor::instance().start();
    MessageProviderManager::instance().setPluginLocator(plugin_locator_);

    core_plugin_manager = std::make_shared<PluginManager<csapex::CorePlugin>>("csapex::CorePlugin");
    node_factory_ = std::make_shared<NodeFactory>(plugin_locator_.get());
    snippet_factory_ = std::make_shared<SnippetFactory>(plugin_locator_.get());

    boot();
}

CsApexCore::CsApexCore(const CsApexCore& parent)
    : CsApexCore(parent.getSettings(), parent.getExceptionHandler(), parent.getPluginLocator())
{
    parent_ = &parent;
    core_plugin_manager = parent.core_plugin_manager;
    node_factory_ =  parent.node_factory_;
    snippet_factory_ =  parent.snippet_factory_;
}

CsApexCore::~CsApexCore()
{
    if(!parent_) {
        root_->clear();
        plugin_locator_->shutdown();
        SingletonInterface::shutdownAll();
        thread_pool_->clear();
    }

    for(std::map<std::string, CorePlugin::Ptr>::iterator it = core_plugins_.begin(); it != core_plugins_.end(); ++it){
        it->second->shutdown();
    }
    core_plugins_.clear();
    core_plugin_manager.reset();

    if(!parent_) {
        boot_plugins_.clear();
        while(!boot_plugin_loaders_.empty()) {
            delete boot_plugin_loaders_.front();
            boot_plugin_loaders_.erase(boot_plugin_loaders_.begin());
        }
    }
}

void CsApexCore::setPause(bool pause)
{
    if(pause != thread_pool_->isPaused()) {
        thread_pool_->setPause(pause);
    }
}

bool CsApexCore::isPaused() const
{
    return thread_pool_->isPaused();
}

bool CsApexCore::isSteppingMode() const
{
    return thread_pool_->isSteppingMode();
}

void CsApexCore::setSteppingMode(bool stepping)
{
    thread_pool_->setSteppingMode(stepping);
}

void CsApexCore::step()
{
    thread_pool_->step();
}

void CsApexCore::setStatusMessage(const std::string &msg)
{
    showStatusMessage(msg);
}

void CsApexCore::init()
{
    if(!init_) {
        init_ = true;

        showStatusMessage("loading core plugins");
        core_plugin_manager->load(plugin_locator_.get());

        for(const auto& cp : core_plugin_manager->getConstructors()) {
            makeCorePlugin(cp.first);
        }

        typedef const std::pair<std::string, CorePlugin::Ptr > PAIR;
        for(PAIR plugin : core_plugins_) {
            plugin.second->prepare(getSettings());
        }
        for(PAIR plugin : core_plugins_) {
            plugin.second->init(*this);
        }

        showStatusMessage("loading node plugins");
        signal_connections_.emplace_back(node_factory_->loaded.connect(showStatusMessage));
        node_factory_->loadPlugins();
        signal_connections_.emplace_back(node_factory_->new_node_type.connect(newNodeType));

        showStatusMessage("make graph");

        root_handle_ = node_factory_->makeNode("csapex::Graph", UUIDProvider::makeUUID_without_parent("~"), root_uuid_provider_.get());
        apex_assert_hard(root_handle_);

        SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(root_handle_->getNode().lock());
        apex_assert_hard(graph);

        root_worker_ = std::make_shared<NodeWorker>(root_handle_);

        root_ = std::make_shared<GraphFacade>(*thread_pool_, graph.get(), root_handle_.get());
        root_->notification.connect(notification);


        root_scheduler_ = std::make_shared<NodeRunner>(root_worker_);
        thread_pool_->add(root_scheduler_.get());

        root_->getSubgraphNode()->createInternalSlot(connection_types::makeEmpty<connection_types::AnyMessage>(),
                                              root_->getGraph()->makeUUID("slot_save"), "save",
                                              [this](const TokenPtr&) {
            saveAs(getSettings().get<std::string>("config"));
        });
        root_->getSubgraphNode()->createInternalSlot(connection_types::makeEmpty<connection_types::AnyMessage>(),
                                              root_->getGraph()->makeUUID("slot_exit"), "exit",
                                              [this](const TokenPtr&) {
            // TODO: more graceful stopping
            csapex::error_handling::stop();
        });
        for(PAIR plugin : core_plugins_) {
            plugin.second->setupGraph(root_->getSubgraphNode());
        }

        showStatusMessage("loading snippets");
        snippet_factory_->loadSnippets();
        signal_connections_.emplace_back(snippet_factory_->snippet_set_changed.connect(newSnippetType));
    }
}

CorePlugin::Ptr CsApexCore::makeCorePlugin(const std::string& plugin_name)
{
    assert(core_plugins_.find(plugin_name) == core_plugins_.end());

    const PluginConstructor<CorePlugin>& constructor = core_plugin_manager->getConstructor(plugin_name);

    CorePlugin::Ptr plugin = constructor();
    plugin->setName(plugin_name);

    core_plugins_[plugin_name] = plugin;

    if(!core_plugins_connected_[plugin_name]) {
        core_plugins_connected_[plugin_name] = true;
    }

    return plugin;
}

void CsApexCore::boot()
{
    showStatusMessage("booting up");

    std::string dir_string = csapex::info::CSAPEX_BOOT_PLUGIN_DIR;

    boost::filesystem::path directory(dir_string);

    if (!boost::filesystem::exists(directory)) {
        return;
    }
    boost::filesystem::directory_iterator dir(directory);
    boost::filesystem::directory_iterator end;

    for(; dir != end; ++dir) {
        boost::filesystem::path path = dir->path();

        boot_plugin_loaders_.push_back(new class_loader::ClassLoader(path.string()));
        class_loader::ClassLoader* loader = boot_plugin_loaders_.back();

        try {
            apex_assert_hard(loader->isLibraryLoaded());
            std::vector<std::string> classes = loader->getAvailableClasses<BootstrapPlugin>();
            for(std::size_t c = 0; c < classes.size(); ++c){
                auto boost_plugin = loader->createInstance<BootstrapPlugin>(classes[c]);
                std::shared_ptr<BootstrapPlugin> plugin = shared_ptr_tools::to_std_shared(boost_plugin);
                boot_plugins_.push_back(plugin);

                plugin->boot(plugin_locator_.get());
            }
        } catch(const std::exception& e) {
            std::cerr << "boot plugin " << path << " failed: " << e.what() << std::endl;
        }
    }

    init();
}

void CsApexCore::startup()
{
    showStatusMessage("loading config");
    try {
        std::string cfg = settings_.get<std::string>("config", Settings::defaultConfigFile());

        bool recovery = settings_.get<bool>("config_recovery", false);
        if(!recovery) {
            load(cfg);

        } else {
            load(settings_.get<std::string>("config_recovery_file"));
            settings_.set("config", cfg);
        }

    } catch(const std::exception& e) {
        std::cerr << "error loading the config: " << e.what() << std::endl;
    }

    root_->getSubgraphNode()->activation();

    showStatusMessage("painting user interface");

    thread_pool_->start();
}

void CsApexCore::reset()
{
    resetRequest();

    root_->clear();

    resetDone();
}

Settings &CsApexCore::getSettings() const
{
    return settings_;
}

NodeFactory &CsApexCore::getNodeFactory() const
{
    return *node_factory_;
}

SnippetFactory& CsApexCore::getSnippetFactory() const
{
    return *snippet_factory_;
}

GraphFacadePtr CsApexCore::getRoot() const
{
    return root_;
}

ThreadPoolPtr CsApexCore::getThreadPool() const
{
    return thread_pool_;
}

PluginLocatorPtr CsApexCore::getPluginLocator() const
{
    return plugin_locator_;
}

ExceptionHandler& CsApexCore::getExceptionHandler() const
{
    return exception_handler_;
}

std::shared_ptr<Profiler> CsApexCore::getProfiler() const
{
    return profiler_;
}

void CsApexCore::settingsChanged()
{
    settings_.save();
    configChanged();
}

void CsApexCore::saveAs(const std::string &file, bool quiet)
{
    std::string dir = file.substr(0, file.find_last_of('/')+1);

    if(!dir.empty()) {
#ifdef WIN32
        int chdir_result = _chdir(dir.c_str());
#else
        int chdir_result = chdir(dir.c_str());
#endif
        if(chdir_result != 0) {
            throw std::runtime_error(std::string("cannot change into directory ") + dir);
        }
    }


    YAML::Node node_map(YAML::NodeType::Map);

    GraphIO graphio(root_->getSubgraphNode(),  node_factory_.get());

    csapex::slim_signal::ScopedConnection connection = graphio.saveViewRequest.connect(settings_.saveDetailRequest);

    settings_.saveRequest(node_map);

    graphio.saveSettings(node_map);

    graphio.saveGraphTo(node_map);

    YAML::Emitter yaml;
    yaml << node_map;

    //    std::cerr << yaml.c_str() << std::endl;

    std::ofstream ofs(file.c_str());
    ofs << "#!" << settings_.get<std::string>("path_to_bin") << '\n';
    ofs << yaml.c_str();

    if(!quiet) {
        saved();
    }
}


void CsApexCore::load(const std::string &file)
{
    settings_.set("config", file);

    bool running = thread_pool_->isRunning();
    if(running) {
        thread_pool_->stop();
    }

    if(load_needs_reset_) {
        reset();
    }

    apex_assert_hard(root_->getGraph()->countNodes() == 0);

    GraphIO graphio(root_->getSubgraphNode(), node_factory_.get());
    graphio.useProfiler(profiler_);

    csapex::slim_signal::ScopedConnection connection = graphio.loadViewRequest.connect(settings_.loadDetailRequest);

    {
        std::ifstream ifs(file.c_str());
        YAML::Parser parser(ifs);

        YAML::NodeBuilder builder;
        if (!parser.HandleNextDocument(builder)) {
            std::cerr << "cannot read the config" << std::endl;
        }
        YAML::Node doc = builder.Root();

        graphio.loadSettings(doc);
        graphio.loadGraphFrom(doc);

        settings_.loadRequest(doc);
    }

    load_needs_reset_ = true;

    loaded();

    if(running) {
        thread_pool_->start();
    }
}
