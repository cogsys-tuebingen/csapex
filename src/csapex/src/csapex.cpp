/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/view/csapex_window.h>
#include <csapex/model/box.h>
#include <csapex/model/boxed_object.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <boost/program_options.hpp>
#include <QtGui>
#include <QApplication>
#include <signal.h>

namespace po = boost::program_options;

using namespace csapex;

void siginthandler(int)
{
    printf("User pressed Ctrl+C\n");
    exit(1);
}

#define DEBUG 0

namespace csapex
{

struct CsApexApp : public QApplication {
    CsApexApp(int& argc, char** argv)
        : QApplication(argc, argv)
    {}

    virtual bool notify(QObject* receiver, QEvent* event) {
        try {
            return QApplication::notify(receiver, event);

        } catch(const std::exception& e) {
            BoxedObject* bo = dynamic_cast<BoxedObject*> (receiver);
            Box* box = dynamic_cast<Box*> (receiver);
            NodeWorker* bw = dynamic_cast<NodeWorker*> (receiver);

            if(bo) {
                bo->setError(true, e.what());
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
};

}

int run(CsApexApp& app)
{
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    signal(SIGINT, siginthandler);
    int result = app.exec();

    return result;
}

int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "show help message")
            ("headless", "run without gui")
            ;

    // filters all qt parameters from argv
    CsApexApp app(argc, argv);

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

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

    CsApexCore core;

    if(!vm.count("headless")) {
        /*
         * There seems to be a bug in Qt4:
         *  A race condition in QApplication sometimes causes a deadlock on startup when using the GTK theme!
         *  Workaround: Specify as a program argument: '-style Plastique' for the Plastique theme or other non-GTK based theme.
         */
        CsApexWindow w(core);
        w.start();

        return run(app);

    } else {
        core.init();
        return run(app);
    }
}

