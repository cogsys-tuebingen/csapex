#ifndef CTRL_MAIN_MENU_H
#define CTRL_MAIN_MENU_H

/// SYSTEM
#include <QMenuBar>
#include <QString>
#include <cmath>

class CtrlMenu : public QObject
{
    Q_OBJECT

public:
    CtrlMenu(QMenuBar *menu);


Q_SIGNALS:
    void loadClassifier(QString path);
    void loadImage(QString path);
    void saveClassifier(QString path);
    void saveClassifierRaw(QString path);
    void saveROIs(QString path);
    void zoom(double factor);

public Q_SLOTS:
    void loadImage();
    void loadClassifier();
    void saveClassifier();
    void saveClassifierRaw();
    void saveROIs();
    void zoomIn();
    void zoomOut();
    void zoomReset();
    void zoomUpdate(double factor);

private:
    QMenuBar *menu_;
    QString   image_path_;
    QString   classifier_path_;
    double    zoom_;

    void snapZoom()
    {
        int step = std::floor(zoom_ / 12.5 + 0.5);
        zoom_ = 12.5 * step;
    }
};

#endif // CTRL_MAIN_MENU_H
