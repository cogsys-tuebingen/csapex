/// HEADER
#include "csapex_client.h"

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/io/server.h>
#include <csapex/io/session_client.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/error_handling.h>
#include <csapex/utility/exceptions.h>
#include <csapex/utility/thread.h>
#include <csapex/view/csapex_view_core_impl.h>
#include <csapex/view/csapex_view_core_proxy.h>
#include <csapex/view/csapex_window.h>
#include <csapex/view/gui_exception_handler.h>

/// SYSTEM
#include <iostream>
#include <QtGui>
#include <QStatusBar>
#include <QMessageBox>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using namespace csapex;

CsApexGuiApp::CsApexGuiApp(int& argc, char** argv, ExceptionHandler& handler) : QApplication(argc, argv), handler(handler)
{
}

CsApexCoreApp::CsApexCoreApp(int& argc, char** argv, ExceptionHandler& handler) : QCoreApplication(argc, argv), handler(handler)
{
}

bool CsApexCoreApp::notify(QObject* receiver, QEvent* event)
{
    try {
        return QCoreApplication::notify(receiver, event);

    } catch (...) {
        std::exception_ptr eptr = std::current_exception();
        handler.handleException(eptr);
        return true;
    }
}

bool CsApexGuiApp::notify(QObject* receiver, QEvent* event)
{
    try {
        return QApplication::notify(receiver, event);

    } catch (...) {
        std::exception_ptr eptr = std::current_exception();
        handler.handleException(eptr);
        return true;
    }
}

Main::Main(QCoreApplication* a, Settings& settings, ExceptionHandler& handler) : app(a), settings(settings), handler(handler), splash(nullptr)
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

    SessionPtr session;
    try {
        std::string host = settings.get<std::string>("host");
        int port = settings.get<int>("port");
        session = std::make_shared<SessionClient>(host, port);

    } catch (const boost::system::system_error& se) {
        std::cerr << "Connection to server failed:\n" << se.what() << std::endl;
        return 1;
    }

    int return_status = 0;
    bool shutdown_server_on_exit = false;
    bool server_has_been_shutdown = false;

    CsApexViewCoreProxy main(session);

    {
        CsApexViewCore& view_core = main;

        CsApexWindow w(view_core);
        w.setWindowIcon(QIcon(":/apex_logo_client.png"));
        QObject::connect(&w, SIGNAL(statusChanged(QString)), this, SLOT(showMessage(QString)));

        app->connect(&w, &CsApexWindow::closed, [&]() {
            if (!server_has_been_shutdown) {
                // we have closed the window -> avoid notification of server shutdown
                view_core.server_shutdown.disconnectAll();

                try {
                    int r = QMessageBox::warning(&w, tr("cs::APEX"), tr("Do you want to stop the server?"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
                    if (r == QMessageBox::Yes) {
                        shutdown_server_on_exit = true;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "exception while stopping graph worker: " << e.what() << std::endl;
                } catch (...) {
                    throw;
                }
            }
            app->quit();
        });

        csapex::error_handling::stop_request().connect([this]() {
            static int request = 0;
            if (request == 0) {
                raise(SIGTERM);
            }

            ++request;
        });

        view_core.server_shutdown.connect([&]() {
            std::cerr << "the server has been shut down." << std::endl;
            server_has_been_shutdown = true;
            w.triggerDisconnectEvent();
        });

        w.start();

        w.show();
        splash->finish(&w);

        return_status = runImpl();
    }

    if (shutdown_server_on_exit) {
        main.shutdown();
    }

    return return_status;
}

int Main::run()
{
    splash = new CsApexSplashScreen;
    splash->show();

    showMessage("loading libraries");

    return runWithGui();
}

void Main::showMessage(const QString& msg)
{
    if (splash->isVisible()) {
        splash->showMessage(msg);
    }
    app->processEvents();
}

int main(int argc, char** argv)
{
    SettingsImplementation settings;

    int effective_argc = argc;
    std::string path_to_bin(argv[0]);

    po::options_description desc("Allowed options");
    desc.add_options()("help", "show help message")("host", po::value<std::string>()->default_value("localhost"), "Host")("port", po::value<int>()->default_value(42123), "Port");
    po::positional_options_description p;

    std::shared_ptr<ExceptionHandler> handler;

    // filters all qt parameters from argv
    std::shared_ptr<GuiExceptionHandler> h(new GuiExceptionHandler(false));

    handler = h;
    QCoreApplication* app = new CsApexGuiApp(effective_argc, argv, *handler);

    h->moveToThread(app->thread());

    // filters ros remappings
    std::vector<std::string> remapping_args;
    std::vector<std::string> rest_args;
    for (int i = 1; i < effective_argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find(":=") != std::string::npos) {
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

    } catch (const std::exception& e) {
        std::cerr << "cannot parse parameters: " << e.what() << std::endl;
        return 4;
    }

    // add ros remappings
    additional_args.insert(additional_args.end(), remapping_args.begin(), remapping_args.end());

    // display help?
    if (vm.count("help")) {
        std::cerr << desc << std::endl;
        return 1;
    }

    // server settings
    settings.set("host", vm["host"].as<std::string>());
    settings.set("port", vm["port"].as<int>());

    // start the app
    Main m(app, settings, *handler);
    try {
        return m.run();

    } catch (const csapex::Failure& af) {
        std::cerr << af.what() << std::endl;
        return 42;
    }
}

/// MOC
#include "moc_csapex_client.cpp"
