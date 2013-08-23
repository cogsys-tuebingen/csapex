#ifndef CTRL_MAIN_MENU_H
#define CTRL_MAIN_MENU_H

/// COMPONENT
#include <gui/terra_trainer_window.h>
#include "controller.hpp"

/// SYSTEM
#include <QMenuBar>
#include <QString>
#include <cmath>

namespace Ui {
class TerraTrainerWindow;
}

class CtrlMainWindow : public QObject, public Controller
{
    Q_OBJECT

public:
    CtrlMainWindow(TerraTrainerWindow *main_window);
    void setupUi(Ui::TerraTrainerWindow *ui);

Q_SIGNALS:
    void zoom(double factor);

public Q_SLOTS:
    void loadImage();
    void loadClassifier();
    void loadSettings();
    void loadROIs();
    void saveSettings();
    void saveClassifierProject();
    void saveClassifierRaw();
    void saveCrops();
    void saveROIs();
    void zoomIn();
    void zoomOut();
    void zoomReset();
    void zoomUpdate(double factor);

private:
    TerraTrainerWindow *main_window_;
    QString             image_path_;
    QString             classifier_path_;
    double              zoom_;

    void snapZoom()
    {
        int step = std::floor(zoom_ / 12.5 + 0.5);
        zoom_ = 12.5 * step;
    }
};

#endif // CTRL_MAIN_MENU_H
