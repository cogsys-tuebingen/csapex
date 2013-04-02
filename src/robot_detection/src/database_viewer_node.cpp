/// PROJECT
#include <config/config.h>
#include <viz/database_viewer.h>

/// SYSTEM
#include <QtGui>
#include <QApplication>
#include <ros/ros.h>
#include <signal.h>

void siginthandler(int param)
{
    printf("User pressed Ctrl+C\n");
    exit(1);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "database_viewer");

    QApplication app(argc, argv);
    DatabaseViewer w;
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    signal(SIGINT, siginthandler);
    int result = app.exec();

    return result;
}
