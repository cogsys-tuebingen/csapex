#ifndef TERRA_TOOLPANEL_WINDOW_H
#define TERRA_TOOLPANEL_WINDOW_H
/// SYSTEM
#include <QMainWindow>

class SubWindow : public QMainWindow
{
    Q_OBJECT

public:
    SubWindow(QWidget *parent = NULL, Qt::WindowFlags flags = Qt::WindowCloseButtonHint | Qt::Tool | Qt::CustomizeWindowHint);

Q_SIGNALS:
    void visible(bool);

protected:
   virtual void closeEvent(QCloseEvent *event);
};

#endif // TERRA_TOOLPANEL_WINDOW_H
