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
        Q_EMIT loadImage(image_path_);
    }
}

void CtrlMenu::loadClassifier()
{
    QString filename = QFileDialog::getOpenFileName(
                0,
                tr("Open Classifier"),
                QDir::currentPath(),
                tr("(*.yaml);;All files (*.*)") );

    if(!filename.isNull()) {
        Q_EMIT loadClassifier(filename);
    }
}

void CtrlMenu::saveClassifier()
{
    QString filename = QFileDialog::getSaveFileName(
                0,
                tr("Save Classifier with class infos!"),
                QDir::currentPath(),
                tr("(*.yaml)") );
    if( !filename.isNull() )
    {
        Q_EMIT saveClassifier(filename);
    }
}

void CtrlMenu::saveClassifierRaw()
{
    QString filename = QFileDialog::getSaveFileName(
                0,
                tr("Save Classifier!"),
                QDir::currentPath(),
                tr("(*.yaml)") );
    if( !filename.isNull() )
    {
        saveClassifierRaw(filename);
    }
}

void CtrlMenu::saveROIs()
{
    QString dir = QFileDialog::getExistingDirectory(
                0,
                tr("Choose directory"),
                QDir::currentPath());
    if(!dir.isNull()) {
        Q_EMIT saveROIs(dir);
    }
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
