#ifndef CTRL_STATE_DISPLAY_H
#define CTRL_STATE_DISPLAY_H
#include <QObject>
#include <boost/any.hpp>

class QProgressDialog;

class QStateDisplay : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<QStateDisplay> Ptr;

    QStateDisplay();


public Q_SLOTS:
    void spawnBar(QString title);
    void stateUpdate(const boost::any &any);
    void despawnBar();

private:
    QProgressDialog *bar_;
};

#endif // CTRL_STATE_DISPLAY_H
