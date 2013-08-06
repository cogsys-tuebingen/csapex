#include "ctrl_main_menu.h"
#include <QFileDialog>
#include <iostream>

CtrlMenu::CtrlMenu(QMenuBar *menu) :
    menu_(menu),
    zoom_(100.0)
{
}

void CtrlMenu::loadImage()
{
    QString filename = QFileDialog::getOpenFileName(
                0,
                tr("Open Image"),
                QDir::currentPath(),
                tr("JPG (*.jpg);; PNG (*.png);; PGM (.pgm);;All files (*.*)") );

    if(!filename.isNull()) {
        image_path_ = filename;
        Q_EMIT imagePath(image_path_);
    }
}

void CtrlMenu::loadClass()
{
    QString filename = QFileDialog::getOpenFileName(
                0,
                tr("Open Classifier"),
                QDir::currentPath(),
                tr("Classifier *.meta.yaml);;All files (*.*)") );

    if(!filename.isNull()) {
        classifier_path_ = filename;
        Q_EMIT classPath(classifier_path_);
    }
}

void CtrlMenu::saveClass()
{

}

void CtrlMenu::zoomIn()
{
    zoom_ += 12.5;
    snapZoom();
    Q_EMIT zoom(zoom_);
}

void CtrlMenu::zoomOut()
{
    zoom_ -= 12.5;
    snapZoom();
    Q_EMIT zoom(zoom_);
}

void CtrlMenu::zoomReset()
{
    zoom_ = 100.0;
    Q_EMIT zoom(zoom_);
}

void CtrlMenu::zoomUpdate(double factor)
{
    zoom_ = factor;
}
