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

/// SYSTEM
#include <fstream>
#include <boost/foreach.hpp>

Q_DECLARE_METATYPE(QSharedPointer<QImage>)
Q_DECLARE_METATYPE(std::string)

using namespace csapex;

CsApexCore::CsApexCore(Settings &settings, GraphWorkerPtr graph, NodeFactory *node_factory, CommandDispatcher* cmd_dispatcher)
    : settings_(settings), graph_worker_(graph), node_factory_(node_factory), cmd_dispatch(cmd_dispatcher), core_plugin_manager(new PluginManager<csapex::CorePlugin>("csapex::CorePlugin")), init_(false)
{
    destruct = true;

    qRegisterMetaType < QSharedPointer<QImage> > ("QSharedPointer<QImage>");
    qRegisterMetaType < std::string > ("std::string");

    StreamInterceptor::instance().start();

    settings.settingsChanged.connect(boost::bind(&CsApexCore::settingsChanged, this));

    QObject::connect(graph_worker_.get(), SIGNAL(paused(bool)), this, SIGNAL(paused(bool)));
}

CsApexCore::~CsApexCore()
{

    for(std::vector<CorePlugin::Ptr>::iterator it = core_plugins_.begin(); it != core_plugins_.end(); ++it){
        (*it)->shutdown();
    }
    core_plugins_.clear();

    StreamInterceptor::instance().stop();

    if(destruct) {
        delete core_plugin_manager;
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

        showStatusMessage("loading core plugins");
        core_plugin_manager->load();
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
        node_factory_->new_box_type.connect(boost::bind(&CsApexCore::reloadBoxMenues, this));
        node_factory_->loadPlugins();
    }
}

void CsApexCore::startup()
{
    showStatusMessage("loading config");
    try {
        load(settings_.getConfig());
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
    int chdir_result = chdir(dir.c_str());
    if(chdir_result != 0) {
        throw std::runtime_error(std::string("cannot change into directory ") + dir);
    }

    YAML::Emitter yaml;

    yaml << YAML::BeginMap; // settings map

    GraphIO graphio(graph_worker_->getGraph(), node_factory_);

    Q_EMIT saveSettingsRequest(yaml);

    graphio.saveSettings(yaml);
    graphio.saveConnections(yaml);

    Q_EMIT saveViewRequest(yaml);

    yaml << YAML::EndMap; // settings map

    graphio.saveNodes(yaml);

    std::ofstream ofs(file.c_str());
    ofs << "#!" << settings_.get<std::string>("path_to_bin") << '\n';
    ofs << yaml.c_str();

    std::cout << "save: " << yaml.c_str() << std::endl;

    cmd_dispatch->setClean();
    cmd_dispatch->resetDirtyPoint();
}


void CsApexCore::load(const std::string &file)
{
    settings_.setCurrentConfig(file);

    reset();

    apex_assert_hard(graph_worker_->getGraph()->countNodes() == 0);

    graph_worker_->setPause(true);

    GraphIO graphio(graph_worker_->getGraph(), node_factory_);

    {
        YAML::Node doc;

        std::ifstream ifs(file.c_str());
        YAML::Parser parser(ifs);

        if(!getNextDocument(parser, doc)) {
            std::cerr << "cannot read the config" << std::endl;
            return;
        }

        Q_EMIT loadSettingsRequest(doc);
        graphio.loadSettings(doc);

        graphio.loadNodes(parser);
    }
    {
        std::ifstream ifs(file.c_str());
        YAML::Parser parser(ifs);

        YAML::Node doc;

        if(!getNextDocument(parser, doc)) {
            std::cerr << "cannot read the config" << std::endl;
            return;
        }
        graphio.loadConnections(doc);

        Q_EMIT loadViewRequest(doc);
    }

    cmd_dispatch->setClean();
    cmd_dispatch->resetDirtyPoint();

    graph_worker_->setPause(false);
}
