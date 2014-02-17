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
#include <csapex/manager/template_manager.h>
#include <utils_plugin/plugin_manager.hpp>
#include <csapex/model/tag.h>

/// SYSTEM
#include <fstream>
#include <boost/foreach.hpp>

using namespace csapex;

Q_DECLARE_METATYPE(QSharedPointer<QImage>)

CsApexCore::CsApexCore(const std::string& config, GraphPtr graph, CommandDispatcher* cmd_dispatcher)
    : graph_(graph), cmd_dispatch(cmd_dispatcher), core_plugin_manager(new PluginManager<csapex::CorePlugin>("csapex::CorePlugin")), init_(false)
{
    destruct = true;

    qRegisterMetaType<QSharedPointer<QImage> >("QSharedPointer<QImage>");
    ConnectionTypeManager::registerMessage<connection_types::AnyMessage> ();
    StreamInterceptor::instance().start();

    Tag::createIfNotExists("General");
    Tag::createIfNotExists("Template");
    Tag::createIfNotExists("Temporary");

    setCurrentConfig(config);
}

CsApexCore::CsApexCore(GraphPtr graph, CommandDispatcher* dispatcher)
    : graph_(graph), cmd_dispatch(dispatcher), core_plugin_manager(new PluginManager<csapex::CorePlugin>("csapex::CorePlugin")), init_(false)
{
    destruct = false;
}


CsApexCore::~CsApexCore()
{
    typedef const std::pair<std::string, PluginManager<CorePlugin>::Constructor> PAIR;
    Q_FOREACH(PAIR cp, core_plugin_manager->availableClasses()) {
        CorePlugin::Ptr plugin = cp.second();

        plugin->shutdown();
    }

    StreamInterceptor::instance().stop();
    BoxManager::instance().stop();
    PluginLoader::instance().stop();

    if(destruct) {
        delete core_plugin_manager;
    }
}

void CsApexCore::setPause(bool pause)
{
    graph_->setPause(pause);
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
        }

        showStatusMessage("loading boxedobject plugins");
        BoxManager& bm = BoxManager::instance();
        bm.loaded.connect(boost::bind(&CsApexCore::showStatusMessage, this, _1));
        bm.new_box_type.connect(boost::bind(&CsApexCore::reloadBoxMenues, this));
        bm.reload();

        showStatusMessage("loading templates");
        // default template path is where custom templates are kept
        TemplateManager::instance().load(TemplateManager::defaultTemplatePath());
        // import plugin templates
        std::vector<std::string> template_paths;
        ros::package::getPlugins("csapex", "template_paths", template_paths);
        BOOST_FOREACH(const std::string& path, template_paths) {
            TemplateManager::instance().load(path);
        }

        showStatusMessage("loading config");
        try {
            load(getConfig());
        } catch(const std::exception& e) {
            std::cerr << "error loading the config: " << e.what() << std::endl;
        }
    }
}

void CsApexCore::setCurrentConfig(const std::string& filename)
{
    current_config_ = filename;

    std::string dir = current_config_.substr(0, current_config_.find_last_of('/')+1);
    chdir(dir.c_str());

    Q_EMIT configChanged();
}

std::string CsApexCore::getConfig() const
{
    return current_config_;
}

void CsApexCore::reset()
{
    cmd_dispatch->reset();

    UUID::reset();

    TemplateManager::instance().reset();

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
    graphio.saveTemplates(yaml);

    yaml << YAML::EndMap; // settings map

    graphio.saveBoxes(yaml);

    std::ofstream ofs(file.c_str());
    ofs << yaml.c_str();

    std::cout << "save: " << yaml.c_str() << std::endl;

    cmd_dispatch->setClean();
    cmd_dispatch->resetDirtyPoint();
}


void CsApexCore::load(const std::string &file)
{
    setCurrentConfig(file);

    reset();

    assert(graph_->countNodes() == 0);

    GraphIO graphio(graph_);

    {
        std::ifstream ifs(file.c_str());
        YAML::Parser parser(ifs);

        YAML::Node doc;

        if(!parser.GetNextDocument(doc)) {
            std::cerr << "cannot read the config" << std::endl;
            return;
        }

        Q_EMIT loadSettingsRequest(doc);
        graphio.loadSettings(doc);
        graphio.loadTemplates(doc);

        graphio.loadBoxes(parser);
    }
    {
        std::ifstream ifs(file.c_str());
        YAML::Parser parser(ifs);

        YAML::Node doc;

        if(!parser.GetNextDocument(doc)) {
            std::cerr << "cannot read the config" << std::endl;
            return;
        }
        graphio.loadConnections(doc);
    }

    cmd_dispatch->setClean();
    cmd_dispatch->resetDirtyPoint();
}
