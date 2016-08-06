/// HEADER
#include <csapex.h>

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/view/designer/drag_io.h>
#include <csapex/core/graphio.h>
#include <csapex/core/settings.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/factory/node_factory.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/node_handle.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/utility/error_handling.h>
#include <csapex/utility/thread.h>
#include <csapex/view/widgets/activity_legend.h>
#include <csapex/view/widgets/activity_timeline.h>
#include <csapex/view/node/box.h>
#include <csapex/view/csapex_window.h>
#include <csapex/view/designer/designer.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/widgets/minimap_widget.h>
#include <csapex/view/node/node_adapter_factory.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/exceptions.h>
#include <csapex/view/gui_exception_handler.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/manager/message_provider_manager.h>
#include <csapex/serialization/serialization.h>

/// SYSTEM
#include <boost/program_options.hpp>
#include <iostream>
#include <QtGui>
#include <QMessageBox>
#include <QStatusBar>
#include <execinfo.h>
#include <stdlib.h>
#include <console_bridge/console.h>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/version.hpp>

#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bf3 = boost::filesystem;
#else
namespace bf3 = boost::filesystem3;
#endif

namespace po = boost::program_options;

using namespace csapex;

CsApexGuiApp::CsApexGuiApp(int& argc, char** argv, ExceptionHandler &handler)
    : QApplication(argc, argv), handler(handler)
{}

CsApexCoreApp::CsApexCoreApp(int& argc, char** argv, ExceptionHandler &handler)
    : QCoreApplication(argc, argv), handler(handler)
{}

bool CsApexGuiApp::doNotify(QObject* receiver, QEvent* event)
{
    return QApplication::notify(receiver, event);
}

bool CsApexCoreApp::doNotify(QObject *receiver, QEvent *event)
{
    return QCoreApplication::notify(receiver, event);
}

bool CsApexCoreApp::notify(QObject* receiver, QEvent* event) {
    return handler.notifyImpl(this, receiver, event);
}
bool CsApexGuiApp::notify(QObject* receiver, QEvent* event) {
    return handler.notifyImpl(this, receiver, event);
}


Main::Main(std::unique_ptr<QCoreApplication> &&a, ExceptionHandler& handler)
    : app(std::move(a)), handler(handler), splash(nullptr)
{
    csapex::thread::set_name("cs::APEX main");
}

Main::~Main()
{
    delete splash;
}

int Main::run()
{
    csapex::error_handling::init();

    int result = app->exec();

    return result;
}

int Main::main(bool headless, bool threadless, bool paused, bool thread_grouping,
               const std::string& config, const std::string& path_to_bin, const std::vector<std::string>& additional_args)
{
    //    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
    if(!headless) {
        splash = new CsApexSplashScreen;
        splash->show();
        showMessage("loading libraries");
    }

    std::string config_to_load = config;

    settings.set("config", config_to_load);
    settings.set("config_recovery", false);

    if(!headless) {
        askForRecoveryConfig(config_to_load);
    }

    if(!settings.knows("path_to_bin")) {
        settings.add(csapex::param::ParameterFactory::declareFileInputPath("path_to_bin", path_to_bin));
    } else {
        settings.set("path_to_bin", path_to_bin);
    }

    if(!settings.knows("headless")) {
        settings.add(csapex::param::ParameterFactory::declareBool("headless", headless));
    } else {
        settings.set("headless", headless);
    }

    if(!settings.knows("threadless")) {
        settings.add(csapex::param::ParameterFactory::declareBool("threadless", threadless));
    } else {
        settings.set("threadless", threadless);
    }

    if(!settings.knows("thread_grouping")) {
        settings.add(csapex::param::ParameterFactory::declareBool("thread_grouping", thread_grouping));
    } else {
        settings.set("thread_grouping", thread_grouping);
    }

    if(!settings.knows("additional_args")) {
        settings.add(csapex::param::ParameterFactory::declareValue< std::vector<std::string> >("additional_args", additional_args));
    } else {
        settings.set("additional_args", additional_args);
    }

    if(!settings.knows("initially_paused")) {
        settings.add(csapex::param::ParameterFactory::declareValue< bool >("initially_paused", paused));
    } else {
        settings.set("initially_paused", paused);
    }

    PluginLocatorPtr plugin_locator =std::make_shared<PluginLocator>(settings);

    CsApexCorePtr core = std::make_shared<CsApexCore>(settings, plugin_locator, handler);

    ThreadPool& thread_pool = *core->getThreadPool();

    settings.saveRequest.connect([&thread_pool](YAML::Node& n){ thread_pool.saveSettings(n); });
    settings.loadRequest.connect([&thread_pool](YAML::Node& n){ thread_pool.loadSettings(n); });


    StreamInterceptor::instance().start();

    MessageProviderManager::instance().setPluginLocator(plugin_locator);

    core->boot();
    core->init();

    int res;
    if(!headless) {
        app->processEvents();

        app->connect(app.get(), SIGNAL(lastWindowClosed()), app.get(), SLOT(quit()));

        GraphFacadePtr root = core->getRoot();

        CommandDispatcher dispatcher(*core);
        csapex::slim_signal::ScopedConnection saved_connection(core->saved.connect([&](){
            dispatcher.setClean();
            dispatcher.resetDirtyPoint();

            bool recovery = settings.get<bool>("config_recovery", false);
            if(recovery) {
                // delete recovery file
                bf3::path recov_file = settings.get<std::string>("config_recovery_file");
                bf3::path current_config  = settings.get<std::string>("config");
                if(recov_file != current_config) {
                    bf3::remove(recov_file);
                    settings.set("config_recovery", false);
                }
            }
        }));
        csapex::slim_signal::ScopedConnection loaded_connection(core->loaded.connect([&](){
            dispatcher.setClean();
            dispatcher.resetDirtyPoint();
        }));
        csapex::slim_signal::ScopedConnection reset(core->resetRequest.connect([&](){
            dispatcher.reset();
            settings.set("config_recovery", false);
        }));


        NodeAdapterFactoryPtr node_adapter_factory = std::make_shared<NodeAdapterFactory>(settings, plugin_locator.get());
        DragIO drag_io(plugin_locator, &dispatcher);

        CsApexViewCore view_core(*core, *node_adapter_factory, dispatcher, drag_io);

        MinimapWidget* minimap = new MinimapWidget;
        Designer* designer = new Designer(view_core, minimap);

        ActivityLegend* legend = new ActivityLegend;
        ActivityTimeline* timeline = new ActivityTimeline;

        QObject::connect(legend, SIGNAL(nodeSelectionChanged(QList<NodeWorker*>)), timeline, SLOT(setSelection(QList<NodeWorker*>)));

        csapex::slim_signal::ScopedConnection add_connection
                (root->nodeWorkerAdded.connect([legend](NodeWorkerPtr n) { legend->startTrackingNode(n); }));
        csapex::slim_signal::ScopedConnection remove_connection
                (root->nodeRemoved.connect([legend](NodeHandlePtr n) { legend->stopTrackingNode(n); }));

        QObject::connect(legend, SIGNAL(nodeAdded(NodeWorker*)), timeline, SLOT(addNode(NodeWorker*)));
        QObject::connect(legend, SIGNAL(nodeRemoved(NodeWorker*)), timeline, SLOT(removeNode(NodeWorker*)));

        CsApexWindow w(*core, &dispatcher, root, thread_pool, designer, minimap, legend, timeline, plugin_locator);
        QObject::connect(&w, SIGNAL(statusChanged(QString)), this, SLOT(showMessage(QString)));

        app->connect(&w, &CsApexWindow::closed, app.get(), &QCoreApplication::quit);

        csapex::error_handling::stop_request().connect([this](){
            static int request = 0;
            if(request++ < 3) {
                std::cout << "shutdown request" << std::endl;
                QCoreApplication::postEvent(app.get(), new QCloseEvent);
            } else {
                raise(SIGTERM);
            }
        });

        node_adapter_factory->loadPlugins();



        bool recover_needed = false;
        QTimer *timer = new QTimer(this);
        QObject::connect(timer, &QTimer::timeout, [&](){
            if(recover_needed) {
                recover_needed = false;
                std::string temp_file_name = settings.get("config")->as<std::string>() + ".recover";
                core->saveAs(temp_file_name, true);
                w.statusBar()->showMessage(tr("Recovery file saved."));
            }
        });
        timer->start(settings.get<int>("config_recovery_save_interval", 1000));
        csapex::slim_signal::ScopedConnection change(dispatcher.stateChanged.connect([&](){
            recover_needed = true;
        }));



        w.start();
        core->startup();

        w.show();
        splash->finish(&w);

        thread_pool.start();

        res = run();

        deleteRecoveryConfig();

        delete designer;

    } else {
        GraphFacadePtr root = core->getRoot();
        csapex::error_handling::stop_request().connect([this, root](){
            app->quit();
        });
        core->startup();

        thread_pool.start();

        res = run();
    }

    StreamInterceptor::instance().stop();

    MessageProviderManager::instance().shutdown();
    Serialization::instance().shutdown();

    return res;
}

void Main::askForRecoveryConfig(const std::string& config_to_load)
{
    bf3::path temp_file = config_to_load + ".recover";
    if(bf3::exists(temp_file)) {
        std::time_t mod_time_t = bf3::last_write_time(temp_file);
        char mod_time[20];
        strftime(mod_time, 20, "%Y-%m-%d %H:%M:%S", localtime(&mod_time_t));

        std::string question = "The application did not exit correctly. "
                               "Do you want to recover<br /><b>" +
                temp_file.filename().string() +
                "</b>?<br />(last modified: " + mod_time + ")";

        QMessageBox::StandardButton reply = QMessageBox::question(splash, "Configuration Recovery",
                                      QString::fromStdString(question),
                                      QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            settings.set("config_recovery", true);
            settings.set("config_recovery_file", temp_file.string());
            settings.set("config_recovery_original", config_to_load);
        }
    }
}

void Main::deleteRecoveryConfig()
{
    bool recovery = settings.get<bool>("config_recovery", false);
    if(!recovery) {
        bf3::path temp_file = settings.get("config")->as<std::string>() + ".recover";
        if(bf3::exists(temp_file)) {
            bf3::remove(temp_file);
        }
    }
}

void Main::showMessage(const QString& msg)
{
    if(splash->isVisible()) {
        splash->showMessage(msg);
    }
    app->processEvents();
}


int main(int argc, char** argv)
{
    //    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
    int effective_argc = argc;
    std::string path_to_bin(argv[0]);

    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "show help message")
            ("dump", "show variables")
            ("paused", "start paused")
            ("headless", "run without gui")
            ("threadless", "run without threading")
            ("fatal_exceptions", "abort execution on exception")
            ("disable_thread_grouping", "by default create one thread per node")
            ("input", "config file to load")
            ;

    po::positional_options_description p;
    p.add("input", 1);

    // first check for --headless or --fatal_exceptions parameter
    // this has to be done before the qapp can be created, which
    // has to be done before parameters can be read.
    bool headless = false;
    bool fatal_exceptions = false;
    for(int i = 1; i < effective_argc; ++i) {
        std::string arg(argv[i]);
        if(arg == "--headless") {
            headless = true;
        } else if(arg == "--fatal_exceptions") {
            fatal_exceptions = true;
        }
    }

    if(!headless) {
        // if headless not requested, check if there is a display
        // if not, we enforce headless mode
        if(!getenv("DISPLAY")) {
            headless = true;
            std::cout << "warning: enforcing headless mode because there is no display detected" << std::endl;
        }
    }


    std::shared_ptr<ExceptionHandler> handler;

    // filters all qt parameters from argv
    std::unique_ptr<QCoreApplication> app;
    if(headless) {
        handler.reset(new ExceptionHandler(fatal_exceptions));
        app.reset(new CsApexCoreApp(effective_argc, argv, *handler));
    } else {
        std::shared_ptr<GuiExceptionHandler> h(new GuiExceptionHandler(fatal_exceptions));

        handler = h;
        app.reset(new CsApexGuiApp(effective_argc, argv, *handler));

        h->moveToThread(app->thread());
    }

    // filters ros remappings
    std::vector<std::string> remapping_args;
    std::vector<std::string> rest_args;
    for(int i = 1; i < effective_argc; ++i) {
        std::string arg(argv[i]);
        if(arg.find(":=") != std::string::npos)  {
            remapping_args.push_back(arg);
        } else {
            rest_args.push_back(arg);
        }
    }

    // now check for remaining parameters
    po::variables_map vm;
    std::vector<std::string> additional_args;

    try {
        po::parsed_options parsed = po::command_line_parser(rest_args).options(desc).positional(p).run();

        po::store(parsed, vm);

        po::notify(vm);

        additional_args = po::collect_unrecognized(parsed.options, po::include_positional);

    } catch(const std::exception& e) {
        std::cerr << "cannot parse parameters: " << e.what() << std::endl;
        return 4;
    }

    // add ros remappings
    additional_args.insert(additional_args.end(), remapping_args.begin(), remapping_args.end());


    // display help?
    if(vm.count("help")) {
        std::cerr << desc << std::endl;
        return 1;
    }
    if(vm.count("dump")) {
        std::cout << "to be passed on:\n";
        for(std::size_t i = 0; i < additional_args.size(); ++i) {
            std::cout << additional_args[i] << "\n";
        }
        std::cout << std::flush;
        return 0;
    }

    // which file to use?
    std::string input;
    if (vm.count("input")) {
        input = vm["input"].as<std::string>();
    } else {
        input = Settings::default_config;
    }

    bool threadless = vm.count("threadless");
    bool thread_grouping = !vm.count("disable_thread_grouping");
    bool paused = vm.count("paused");

    // start the app
    Main m(std::move(app), *handler);
    try {
        return m.main(headless, threadless, paused, thread_grouping, input, path_to_bin, additional_args);

    } catch(const csapex::Failure& af) {
        std::cerr << af.what() << std::endl;
        return 42;
    }
}

/// MOC
#include "moc_csapex.cpp"
