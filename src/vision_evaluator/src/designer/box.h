#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include "connector_in.h"
#include "connector_out.h"
#include "memento.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QWidget>
#include <yaml-cpp/yaml.h>

/// FORWARD DECLARATIONS
namespace Ui
{
class Box;
}


namespace vision_evaluator
{

class BoxedObject;

class Box : public QWidget
{
    Q_OBJECT

public:
    struct MoveOffset : public QObjectUserData {
        MoveOffset(const QPoint& o)
            : value(o)
        {}

        QPoint value;
    };

public:
    static const QString MIME;
    static const QString MIME_MOVE;


public:
    Box(BoxedObject* content, const std::string& uuid = "", QWidget* parent = 0);
    virtual ~Box();

    void stop();

    virtual void mousePressEvent(QMouseEvent* e);
    virtual QPixmap makePixmap(const std::string& label);

    void moveEvent(QMoveEvent*);

    virtual void init(const QPoint& pos);
    BoxedObject* getContent();

    void addInput(ConnectorIn* in);
    void addOutput(ConnectorOut* out);

    ConnectorIn* getInput(const std::string& uuid);
    ConnectorOut* getOutput(const std::string& uuid);

    void setUUID(const std::string& uuid);
    std::string UUID() const;

    Memento::Ptr getState() const;
    void setState(Memento::Ptr state);

    YAML::Emitter& save(YAML::Emitter& out) const;

protected:
    void startDrag(QPoint offset);
    void paintEvent(QPaintEvent* e);
    bool eventFilter(QObject*, QEvent*);
    void enabledChange(bool val);

public Q_SLOTS:
    virtual void setOverlay(Overlay* o);
    void showContextMenu(const QPoint& pos);

Q_SIGNALS:
    void toggled(bool);
    void moved(Box*);

private:
    Ui::Box* ui;

    std::vector<ConnectorIn*> input;
    std::vector<ConnectorOut*> output;

    bool down_;

    BoxedObject* content_;
    Overlay* overlay_;

    std::string uuid_;
};

inline YAML::Emitter& operator << (YAML::Emitter& out, const Box& box)
{
    return box.save(out);
}

}
#endif // BOX_H
