/// HEADER
#include <csapex.h>

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/core/settings.h>
#include <csapex/view/csapex_window.h>
#include <csapex/view/box.h>
#include <csapex/model/node_worker.h>
#include <csapex/core/graphio.h>
#include <csapex/utility/thread.h>
#include <csapex/utility/error_handling.h>
#include <csapex/core/drag_io.h>
#include <csapex/view/designer.h>
#include <csapex/view/design_board.h>
#include <csapex/manager/box_manager.h>
#include <utils_param/parameter_factory.h>
#include <csapex/view/widget_controller.h>
#include <csapex/view/overlay.h>

/// SYSTEM
#include <boost/program_options.hpp>
#include <QtGui>
#include <execinfo.h>
#include <stdlib.h>

namespace po = boost::program_options;

using namespace csapex;


CsApexApp::CsApexApp(int& argc, char** argv)
    : QApplication(argc, argv)
{}

bool CsApexApp::notify(QObject* receiver, QEvent* event) {
    try {
        return QApplication::notify(receiver, event);

    } catch(const std::exception& e) {
        ErrorState* er = dynamic_cast<ErrorState*> (receiver);
        Box* box = dynamic_cast<Box*> (receiver);
        NodeWorker* bw = dynamic_cast<NodeWorker*> (receiver);

        if(er) {
            er->setError(true, e.what());
        } else if(box) {
            box->setError(true, e.what());
        } else if(bw) {
            bw->triggerError(true, e.what());
        } else {
            std::cerr << "Uncatched exception:" << e.what() << std::endl;
#if DEBUG
            throw;
#endif
        }

        return false;

    } catch(const std::string& s) {
        std::cerr << "Uncatched exception (string) exception: " << s << std::endl;
#if DEBUG
        throw;
#endif
    } catch(...) {
        std::cerr << "Uncatched exception of unknown type and origin!" << std::endl;
        throw;
    }

    return true;
}

Main::Main(CsApexApp& a)
    : app(a), splash(NULL)
{
    csapex::thread::set_name("cs::APEX main");
}

int Main::run()
{
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    csapex::error_handling::init();

    int result = app.exec();

    return result;
}

int Main::main(bool headless, const std::string& config, const std::string& path_to_bin)
{
    Settings settings;
    settings.setCurrentConfig(config);

    if(!settings.knows("path_to_bin")) {
        settings.add(param::ParameterFactory::declarePath("path_to_bin", path_to_bin));
    } else {
        settings.set("path_to_bin", path_to_bin);
    }

    BoxManager::instance().settings_ = &settings;

    Graph::Ptr graph(new Graph(settings));
    WidgetControllerPtr widget_control (new WidgetController(graph));

    CommandDispatcher dispatcher(graph, widget_control);

    CsApexCore core(settings, graph, &dispatcher);

    if(!headless) {
        /*
             * There seems to be a bug in Qt4:
             *  A race condition in QApplication sometimes causes a deadlock on startup when using the GTK theme!
             *  Workaround: Specify as a program argument: '-style Plastique' for the Plastique theme or other non-GTK based theme.
             */
        QPixmap pm(":/apex_splash.png");
        splash = new QSplashScreen(pm);
        splash->show();

        app.processEvents();

        DragIO drag_io(graph.get(), &dispatcher, widget_control);
        Overlay* overlay   = new Overlay(graph, &dispatcher, widget_control);
        DesignBoard* board = new DesignBoard(graph, &dispatcher, widget_control, drag_io, overlay);
        Designer* designer = new Designer(settings, graph, &dispatcher, widget_control, board);

        widget_control->setDesigner(designer);

        CsApexWindow w(core, &dispatcher, widget_control, graph, designer);
        QObject::connect(&w, SIGNAL(statusChanged(QString)), this, SLOT(showMessage(QString)));
        w.start();

        core.init(&drag_io);

        splash->finish(&w);
        w.setVisible(true);

        int res = run();

        delete splash;

        return res;

    } else {
        core.init(NULL);
        return run();
    }
}

void Main::showMessage(const QString& msg)
{
    if(splash->isVisible()) {
        splash->showMessage(msg, Qt::AlignBottom | Qt::AlignRight, Qt::black);
    }
    app.processEvents();
}


int main(int argc, char** argv)
{
    std::string path_to_bin(argv[0]);

    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "show help message")
            ("headless", "run without gui")
            ("input", "config file to load")
            ;

    po::positional_options_description p;
    p.add("input", -1);

    // filters all qt parameters from argv
    CsApexApp app(argc, argv);

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    } catch(po::unknown_option& e) {
        std::cerr << "Error parsing parameters: " << e.what() << "\n";
        std::cerr << desc << std::endl;
        return 2;
    }

    po::notify(vm);

    if(vm.count("help")) {
        std::cerr << desc << std::endl;
        return 1;
    }

    std::string input;
    if (vm.count("input")) {
        input = vm["input"].as<std::string>();
    } else {
        input = Settings::default_config;
    }

    Main m(app);
    return m.main(vm.count("headless"), input, path_to_bin);
}

