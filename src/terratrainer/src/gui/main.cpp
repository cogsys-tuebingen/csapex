#include "terra_trainer_window.h"
#include <QApplication>
#include <QRectF>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TerraTrainerWindow w;
    w.show();
    return a.exec();
}

/*************
 * boxes to select
 * - rotatable

 * render Grid
 * render Quadtree segmentation
 *
 * training button
 * current training input textarea
 *
 **************/
