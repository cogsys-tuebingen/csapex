/// COMPONENT
#include "background_remover_node.h"
#include "background_remover_window.h"

/// SYSTEM
#include <QApplication>
#include <QtGui>
#include <ros/ros.h>
#include <signal.h>

using namespace background_subtraction;

void siginthandler(int param)
{
    printf("User pressed Ctrl+C\n");
    exit(1);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "bg_remove");
    ros::NodeHandle nh("~");

    if(argc > 1 && std::string(argv[1]) == "--no-gui") {
        QMutex mutex;
        BackgroundRemoverNode node(nh, mutex);
        node.init();
        node.wait();
        return 0;

    } else {
        QApplication app(argc, argv);
        BackgroundRemoverWindow w(nh);
        w.show();
        app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
        app.connect(&app, SIGNAL(lastWindowClosed()), &w, SLOT(shutdown()));
        signal(SIGINT, siginthandler);
        int result = app.exec();

        return result;
    }
}
