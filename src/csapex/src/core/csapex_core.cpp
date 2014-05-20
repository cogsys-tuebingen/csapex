/// HEADER
#include <csapex/core/csapex_core.h>

/// COMPONENT
#include <csapex/manager/box_manager.h>
#include <csapex/core/core_plugin.h>
#include <csapex/core/graphio.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/command/dispatcher.h>
#include <csapex/model/message.h>
#include <csapex/manager/connection_type_manager.h>
#include <utils_plugin/plugin_manager.hpp>
#include <csapex/model/tag.h>

/// SYSTEM
#include <fstream>
#include <boost/foreach.hpp>

using namespace csapex;

Q_DECLARE_METATYPE(QSharedPointer<QImage>)
Q_DECLARE_METATYPE(std::string)

CsApexCore::CsApexCore(Settings &settings, GraphPtr graph, CommandDispatcher* cmd_dispatcher)
    : settings_(settings), graph_(graph), cmd_dispatch(cmd_dispatcher), core_plugin_manager(new PluginManager<csapex::CorePlugin>("csapex::CorePlugin")), init_(false)
{
    destruct = true;

    qRegisterMetaType < QSharedPointer<QImage> > ("QSharedPointer<QImage>");
    qRegisterMetaType < std::string > ("std::string");

    ConnectionTypeManager::registerMessage<connection_types::AnyMessage> ();
    StreamInterceptor::instance().start();

    Tag::createIfNotExists("General");
    Tag::createIfNotExists("Template");
    Tag::createIfNotExists("Temporary");

    settings.settingsChanged.connect(boost::bind(&CsApexCore::settingsChanged, this));

    QObject::connect(graph.get(), SIGNAL(paused(bool)), this, SIGNAL(paused(bool)));
}

CsApexCore::~CsApexCore()
{

    for(std::vector<CorePlugin::Ptr>::iterator it = core_plugins_.begin(); it != core_plugins_.end(); ++it){
        (*it)->shutdown();
    }
    core_plugins_.clear();

    StreamInterceptor::instance().stop();
    BoxManager::instance().stop();
    PluginLoader::instance().stop();

    if(destruct) {
        delete core_plugin_manager;
    }
}

void CsApexCore::setPause(bool pause)
{
    if(pause != graph_->isPaused()) {
        std::cout << (pause ? "pause" : "unpause") << std::endl;
        graph_->setPause(pause);
    }
}

bool CsApexCore::isPaused() const
{
    return graph_->isPaused();
}

void CsApexCore::clearBlock()
{
    graph_->clearBlock();
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
        core_plugin_manager->reload();
        typedef const std::pair<std::string, PluginManager<CorePlugin>::Constructor> PAIR;
        Q_FOREACH(PAIR cp, core_plugin_manager->availableClasses()) {
            CorePlugin::Ptr plugin = cp.second();

            plugin->init(*this);

            if(dragio) {
                plugin->initUI(*dragio);
            }

            core_plugins_.push_back(plugin);
        }

        showStatusMessage("loading node plugins");
        BoxManager& bm = BoxManager::instance();
        bm.loaded.connect(boost::bind(&CsApexCore::showStatusMessage, this, _1));
        bm.new_box_type.connect(boost::bind(&CsApexCore::reloadBoxMenues, this));
        bm.reload();

        showStatusMessage("loading config");
        try {
            load(settings_.getConfig());
        } catch(const std::exception& e) {
            std::cerr << "error loading the config: " << e.what() << std::endl;
        }
        showStatusMessage("painting user interface");
    }
}

void CsApexCore::reset()
{
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

void CsApexCore::settingsChanged()
{
    settings_.save();
    Q_EMIT configChanged();
}

void CsApexCore::saveAs(const std::string &file)
{
    std::string dir = file.substr(0, file.find_last_of('/')+1);
    chdir(dir.c_str());

    YAML::Emitter yaml;

    yaml << YAML::BeginMap; // settings map

    GraphIO graphio(graph_);

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

    assert(graph_->countNodes() == 0);

    GraphIO graphio(graph_);

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
}
