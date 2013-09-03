#ifndef ANALYZER_NODE_H
#define ANALYZER_NODE_H

/// PROJECT
#include <config/reconfigurable.h>
#include <data/frame.h>

/// SYSTEM
#include <QApplication>
#include <signal.h>

/// FORWARD DECLARATION
class Analyzer;
class QMainWindow;

namespace
{
void siginthandler(int /*param*/)
{
    WARN("User pressed Ctrl+C");
    exit(1);
}
}

/**
 * @brief The AnalyzerAdapter class is a base class for Analyzer adapters.
 */
class AnalyzerAdapter : public Reconfigurable
{
protected:
    /**
     * @brief AnalyzerAdapter
     * @param analyzer The analyzer to wrap
     * @throws if something went wrong
     */
    AnalyzerAdapter(Analyzer& analyzer);

public:
    /**
     * @brief ~AnalyzerAdapter
     */
    virtual ~AnalyzerAdapter();

    /**
     * @brief run Runs the main loop in a QtWindow until a shutdown is requested
     * @param argc Parameters for QtGui
     * @param argv Parameters for QtGui
     * @return exit code
     */
    template <class WindowType>
    int run(int argc, char** argv) {
        bool gui = config("gui_enabled") && (argc <= 1 || std::string(argv[1]) != "--no-gui");

        if(!gui) {
            runHeadless();
            return 0;

        } else {
            QApplication app(argc, argv);
            WindowType w(this);
            w.show();
            app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
            app.connect(&app, SIGNAL(lastWindowClosed()), &w, SLOT(shutdown()));
            signal(SIGINT, siginthandler);
            int result = app.exec();

            return result;
        }
    }

    /**
     * @brief runHeadless Runs the main loop without a QtWindow
     */
    virtual void runHeadless();

    /**
     * @brief getAnalyzer accessor
     * @return The adaptee
     */
    Analyzer& getAnalyzer() {
        return analyzer;
    }

    /**
     * @brief shutdown Stop all execution
     */
    virtual void shutdown() {}

protected:
    Analyzer& analyzer;
};

#endif // ANALYZER_NODE_H
