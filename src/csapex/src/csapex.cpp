/// HEADER
#include <csapex.h>

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/core/drag_io.h>
#include <csapex/core/graphio.h>
#include <csapex/core/settings.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/node_worker.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/utility/error_handling.h>
#include <csapex/utility/thread.h>
#include <csapex/view/activity_legend.h>
#include <csapex/view/activity_timeline.h>
#include <csapex/view/box.h>
#include <csapex/view/csapex_window.h>
#include <csapex/view/designer.h>
#include <csapex/view/designer_scene.h>
#include <csapex/view/designer_view.h>
#include <csapex/view/minimap_widget.h>
#include <csapex/view/node_adapter_factory.h>
#include <csapex/view/widget_controller.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <boost/program_options.hpp>
#include <QtGui>
#include <execinfo.h>
#include <stdlib.h>
#include <console_bridge/console.h>

namespace po = boost::program_options;

using namespace csapex;


CsApexApp::CsApexApp(int& argc, char** argv, bool fatal_exceptions)
    : QApplication(argc, argv), fatal_exceptions_(fatal_exceptions)
{}

CsApexCoreApp::CsApexCoreApp(int& argc, char** argv, bool fatal_exceptions)
    : QCoreApplication(argc, argv), fatal_exceptions_(fatal_exceptions)
{}

bool CsApexApp::notify(QObject* receiver, QEvent* event) {
    try {
        return QApplication::notify(receiver, event);

    } catch(const std::exception& e) {
        ErrorState* er = dynamic_cast<ErrorState*> (receiver);
        NodeWorker* bw = dynamic_cast<NodeWorker*> (receiver);

        if(fatal_exceptions_) {
            std::cerr << "caught an exception in --fatal-exceptions mode: Abort!" << std::endl;
            std::abort();
        }

        std::cerr << "exception: " << e.what() << std::endl;

        if(er) {
            er->setError(true, e.what());
        } else if(bw) {
            bw->triggerError(true, e.what());
        } else {
            std::cerr << "Uncatched exception:" << e.what() << std::endl;
        }

        return false;

    } catch(const std::string& s) {
        std::cerr << "Uncatched exception (string) exception: " << s << std::endl;
    } catch(...) {
        std::cerr << "Uncatched exception of unknown type and origin!" << std::endl;
        std::abort();
    }

    return true;
}

bool CsApexCoreApp::notify(QObject* receiver, QEvent* event) {
    try {
        return QCoreApplication::notify(receiver, event);

    } catch(const std::exception& e) {
        ErrorState* er = dynamic_cast<ErrorState*> (receiver);
        NodeWorker* bw = dynamic_cast<NodeWorker*> (receiver);

        if(fatal_exceptions_) {
            std::cerr << "caught an exception in --fatal-exceptions mode: Abort!" << std::endl;
            std::abort();
        }

        std::cerr << "exception: " << e.what() << std::endl;

        if(er) {
            er->setError(true, e.what());
        } else if(bw) {
            bw->triggerError(true, e.what());
        } else {
            std::cerr << "Uncatched exception:" << e.what() << std::endl;
        }

        return false;

    } catch(const std::string& s) {
        std::cerr << "Uncatched exception (string) exception: " << s << std::endl;
    } catch(...) {
        std::cerr << "Uncatched exception of unknown type and origin!" << std::endl;
        std::abort();
    }

    return true;
}

Main::Main(std::unique_ptr<QCoreApplication> &&a)
    : app(std::move(a)), splash(nullptr)
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

int Main::main(bool headless, bool threadless, bool paused, bool thread_grouping, const std::string& config, const std::string& path_to_bin, const std::vector<std::string>& additional_args)
{
//    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
    if(!headless) {
        splash = new CsApexSplashScreen;
        splash->show();
        showMessage("loading libraries");
    }

    Settings settings;
    settings.set("config", config);

    if(!settings.knows("path_to_bin")) {
        settings.add(param::ParameterFactory::declareFileInputPath("path_to_bin", path_to_bin));
    } else {
        settings.set("path_to_bin", path_to_bin);
    }

    if(!settings.knows("headless")) {
        settings.add(param::ParameterFactory::declareBool("headless", headless));
    } else {
        settings.set("headless", headless);
    }

    if(!settings.knows("threadless")) {
        settings.add(param::ParameterFactory::declareBool("threadless", threadless));
    } else {
        settings.set("threadless", threadless);
    }

    if(!settings.knows("thread_grouping")) {
        settings.add(param::ParameterFactory::declareBool("thread_grouping", thread_grouping));
    } else {
        settings.set("thread_grouping", thread_grouping);
    }

    if(!settings.knows("additional_args")) {
        settings.add(param::ParameterFactory::declareValue< std::vector<std::string> >("additional_args", additional_args));
    } else {
        settings.set("additional_args", additional_args);
    }

    PluginLocatorPtr plugin_locator =std::make_shared<PluginLocator>(settings);


    GraphPtr graph = std::make_shared<Graph>();
    ThreadPool thread_pool(!threadless, thread_grouping, paused);
    GraphWorkerPtr graph_worker = std::make_shared<GraphWorker>(thread_pool, graph.get());

    graph_worker->generatorAdded.connect([&thread_pool](TaskGeneratorPtr tg) {
        thread_pool.add(tg.get());
    });
    graph_worker->generatorRemoved.connect([&thread_pool](TaskGeneratorPtr tg) {
        thread_pool.remove(tg.get());
    });
    graph_worker->paused.connect([&thread_pool](bool pause) {
        thread_pool.setPause(pause);
    });
    graph_worker->stopped.connect([&thread_pool]() {
        thread_pool.stop();
    });

    NodeFactoryPtr node_factory = std::make_shared<NodeFactory>(settings, plugin_locator.get());
    NodeAdapterFactoryPtr node_adapter_factory = std::make_shared<NodeAdapterFactory>(settings, plugin_locator.get());
    CommandDispatcher dispatcher(settings, graph_worker, graph, &thread_pool, node_factory.get());

    // TODO: core must be destroyed AFTER the factories -> refactor it
    CsApexCorePtr core = std::make_shared<CsApexCore>(settings, plugin_locator,
                                                      graph_worker, graph,
                                                      thread_pool, node_factory.get(), node_adapter_factory.get(), &dispatcher);

    core->saveSettingsRequest.connect([&thread_pool](YAML::Node& n){ thread_pool.saveSettings(n); });
    core->loadSettingsRequest.connect([&thread_pool](YAML::Node& n){ thread_pool.loadSettings(n); });


    int res;
    if(!headless) {
        app->processEvents();

        app->connect(app.get(), SIGNAL(lastWindowClosed()), app.get(), SLOT(quit()));

        WidgetControllerPtr widget_control = std::make_shared<WidgetController>(settings, dispatcher, graph_worker, node_factory.get(), node_adapter_factory.get());
        DragIO drag_io(graph.get(), &dispatcher, widget_control);

        DesignerStyleable style;
        DesignerScene* scene = new DesignerScene(graph, &dispatcher, widget_control, &style);
        DesignerView* view = new DesignerView(scene, graph, settings, thread_pool, &dispatcher, widget_control, drag_io, &style);
        MinimapWidget* minimap = new MinimapWidget(view, scene);

        Designer* designer = new Designer(settings, graph, &dispatcher, widget_control, view, scene, minimap);

        ActivityLegend* legend = new ActivityLegend;
        ActivityTimeline* timeline = new ActivityTimeline;

        QObject::connect(legend, SIGNAL(nodeSelectionChanged(QList<NodeWorker*>)), timeline, SLOT(setSelection(QList<NodeWorker*>)));

        graph->nodeAdded.connect([legend](NodeWorkerPtr n) { legend->startTrackingNode(n); });
        graph->nodeRemoved.connect([legend](NodeWorkerPtr n) { legend->stopTrackingNode(n); });

        QObject::connect(legend, SIGNAL(nodeAdded(NodeWorker*)), timeline, SLOT(addNode(NodeWorker*)));
        QObject::connect(legend, SIGNAL(nodeRemoved(NodeWorker*)), timeline, SLOT(removeNode(NodeWorker*)));

        widget_control->setDesigner(designer);

        CsApexWindow w(*core, &dispatcher, widget_control,
                       graph_worker, graph,
                       thread_pool, designer, minimap, legend, timeline, plugin_locator.get());
        QObject::connect(&w, SIGNAL(statusChanged(QString)), this, SLOT(showMessage(QString)));

        csapex::error_handling::stop_request().connect(std::bind(&CsApexWindow::close, &w));

        core->init(&drag_io);
        w.start();
        core->startup();

        w.show();
        splash->finish(&w);

        res = run();

        // FIXME: this is necessary for some adapted nodes.
        graph->reset();
        delete designer;

    } else {
        core->init(nullptr);
        csapex::error_handling::stop_request().connect(std::bind(&csapex::error_handling::kill));
        core->startup();
        res = run();
    }

    node_factory.reset();
    node_adapter_factory.reset();

    return res;
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

    std::string path_to_bin(argv[0]);

    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "show help message")
            ("dump", "show variables")
            ("paused", "start paused")
            ("headless", "run without gui")
            ("threadless", "run without threading")
            ("fatal_exceptions", "abort execution on exception")
            ("thread_grouping", "create one thread per graph component")
            ("input", "config file to load")
            ;

    po::positional_options_description p;
    p.add("input", 1);

    // first check for --headless or --fatal_exceptions parameter
    // this has to be done before the qapp can be created, which
    // has to be done before parameters can be read.
    bool headless = false;
    bool fatal_exceptions = false;
    for(int i = 1; i < argc; ++i) {
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

    // filters all qt parameters from argv
    std::unique_ptr<QCoreApplication> app;
    if(headless) {
        app.reset(new CsApexCoreApp(argc, argv, fatal_exceptions));
    } else {
        app.reset(new CsApexApp(argc, argv, fatal_exceptions));
    }

    // filters ros remappings
    std::vector<std::string> remapping_args;
    std::vector<std::string> rest_args;
    for(int i = 1; i < argc; ++i) {
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
    bool thread_grouping = vm.count("thread_grouping");
    bool paused = vm.count("paused");

    // start the app
    Main m(std::move(app));
    return m.main(headless, threadless, paused, thread_grouping, input, path_to_bin, additional_args);
}

/// MOC
#include "moc_csapex.cpp"
