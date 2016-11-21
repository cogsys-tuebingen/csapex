/// HEADER
#include <csapex.h>

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/core/settings.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/error_handling.h>
#include <csapex/utility/exceptions.h>
#include <csapex/utility/thread.h>
#include <csapex/view/csapex_view_core.h>
#include <csapex/view/csapex_window.h>
#include <csapex/view/gui_exception_handler.h>

/// SYSTEM
#include <iostream>
#include <QtGui>
#include <QStatusBar>
#include <QMessageBox>
#include <boost/program_options.hpp>
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


Main::Main(std::unique_ptr<QCoreApplication> &&a, Settings& settings, ExceptionHandler& handler)
    : app(std::move(a)), settings(settings), handler(handler), splash(nullptr), recover_needed(false)
{
    csapex::thread::set_name("cs::APEX main");
}

Main::~Main()
{
    delete splash;
}

int Main::runImpl()
{
    csapex::error_handling::init();

    int result = app->exec();

    return result;
}

int Main::runWithGui()
{
    app->processEvents();

    CsApexViewCore view_core(*core);

    CsApexWindow w(view_core);
    QObject::connect(&w, SIGNAL(statusChanged(QString)), this, SLOT(showMessage(QString)));

    app->connect(&w, &CsApexWindow::closed, app.get(), &QCoreApplication::quit);
    app->connect(app.get(), SIGNAL(lastWindowClosed()), app.get(), SLOT(quit()));

    csapex::error_handling::stop_request().connect([this](){
        static int request = 0;
        if(request++ < 3) {
            std::cout << "shutdown request" << std::endl;
            QCoreApplication::postEvent(app.get(), new QCloseEvent);
        } else {
            raise(SIGTERM);
        }
    });

    checkRecoveryFile(view_core, w);

    w.start();
    core->startup();

    w.show();
    splash->finish(&w);

    int res = runImpl();

    deleteRecoveryConfig();

    return res;
}

int Main::runHeadless()
{
    GraphFacadePtr root = core->getRoot();
    csapex::error_handling::stop_request().connect([this, root](){
        app->quit();
    });
    core->startup();

    return runImpl();
}

int Main::run()
{
    bool headless = settings.get<bool>("headless");

    std::string config_to_load = settings.get<std::string>("config");

    if(!headless) {
        splash = new CsApexSplashScreen;
        splash->show();

        settings.set("config_recovery", false);
        askForRecoveryConfig(config_to_load);

        showMessage("loading libraries");
    }

    core = std::make_shared<CsApexCore>(settings, handler);

    if(headless) {
        return runHeadless();
    } else {
        return runWithGui();
    }
}

void Main::checkRecoveryFile(CsApexViewCore &view_core, CsApexWindow &w)
{
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
    observe(view_core.getCommandDispatcher().state_changed, [&](){
        recover_needed = true;
    });
}

void Main::askForRecoveryConfig(const std::string& config_to_load)
{
    bf3::path temp_file = config_to_load + ".recover";
    if(bf3::exists(temp_file)) {
        showMessage("handling recovery file");

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
    Settings settings;

    int effective_argc = argc;
    std::string path_to_bin(argv[0]);

    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "show help message")
            ("debug", "enable debug output")
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
#if WIN32
        if (false) {
#else
        if (!getenv("DISPLAY")) {
#endif
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
            std::cout << additional_args[i] << '\n';
        }
        std::cout << std::flush;
        return 0;
    }

    // which file to use?
    if (vm.count("input")) {
        settings.set("config",vm["input"].as<std::string>());
    } else {
        settings.set("config",Settings::default_config);
    }

    if(!settings.knows("path_to_bin")) {
        settings.add(csapex::param::ParameterFactory::declareFileInputPath("path_to_bin", path_to_bin));
    } else {
        settings.set("path_to_bin", path_to_bin);
    }

    settings.set("debug", vm.count("debug") > 0);
    settings.set("headless", headless);
    settings.set("threadless", vm.count("threadless") > 0);
    settings.set("thread_grouping", vm.count("disable_thread_grouping") == 0);
    settings.set("additional_args", additional_args);
    settings.set("initially_paused", vm.count("paused") > 0);

    // start the app
    Main m(std::move(app), settings, *handler);
    try {
        return m.run();

    } catch(const csapex::Failure& af) {
        std::cerr << af.what() << std::endl;
        return 42;
    }
}

/// MOC
#include "moc_csapex.cpp"
