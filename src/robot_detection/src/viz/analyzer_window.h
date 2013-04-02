#ifndef ANALYZER_WINDOW_H
#define ANALYZER_WINDOW_H

/// COMPONENT
#include "database_io_window.h"

/// PROJECT
#include <data/frame.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <QGraphicsScene>
#include <QThread>
#include <opencv2/core/core.hpp>

/// FORWARD DECLARATION
class Analyzer;
class AnalyzerAdapter;

namespace Ui
{
class AnalyzerWindow;
}

/**
 * @brief The AnalyzerThread class spinns an Analyzer asynchronously
 */
class AnalyzerThread : public QThread
{
public:
    AnalyzerThread(AnalyzerAdapter& analyzer)
        : analyzer_adapter(analyzer) {}

    void run();

private:
    AnalyzerAdapter& analyzer_adapter;
};

/**
 * @brief The AnalyzerWindow class visualizes an arbitrary Analyzer
 */
class AnalyzerWindow : public DatabaseIOWindow
{
    Q_OBJECT

public:
    /**
     * @brief AnalyzerWindow
     * @param analyzer_node the Analyzer
     * @param parent
     */
    explicit AnalyzerWindow(AnalyzerAdapter* analyzer_adapter, QWidget* parent = 0);

    /**
     * @brief ~AnalyzerWindow
     */
    virtual ~AnalyzerWindow();

Q_SIGNALS:
    void updateViewRequest(Frame::Ptr current_frame);

public Q_SLOTS:
    void updateView(Frame::Ptr current_frame);
    void changeState();
    void shutdown();

private:
    void triggerUpdateViewRequest(Frame::Ptr current_frame);
    void setView(int index, cv::Mat& i, const std::string& name);

private:
    Ui::AnalyzerWindow* ui;

    std::vector<QGraphicsScene*> scenes;
    std::vector<QGraphicsView*> tabs;

    Analyzer& analyzer;
    AnalyzerAdapter* analyzer_adapter;
    AnalyzerThread* thread;

    boost::signals2::connection analyzer_connection;

    bool show_controls;
};

#endif // ANALYZER_WINDOW_H
