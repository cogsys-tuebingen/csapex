#ifndef OUTPUT_DISPLAY_H
#define OUTPUT_DISPLAY_H

/// PROJECT
#include <designer/boxed_object.h>

/// SYSTEM
#include <QGraphicsView>

namespace vision_evaluator
{

class ConnectorIn;

class OutputDisplay : public BoxedObject
{
    Q_OBJECT

public:
    OutputDisplay();
    virtual ~OutputDisplay();

    virtual void fill(QBoxLayout* layout);

    virtual void enable();
    virtual void disable();
    virtual void connectorChanged();

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    void messageArrived(ConnectorIn* source);
    void display(QSharedPointer<QImage> img);

Q_SIGNALS:
    void displayRequest(QSharedPointer<QImage> img);

protected:
    bool eventFilter(QObject* o, QEvent* e);

private:

    struct State : public Memento {
        int width;
        int height;

        State()
            : width(300), height(300)
        {}

        virtual void writeYaml(YAML::Emitter& out) const {
            out << YAML::Key << "width" << YAML::Value << width;
            out << YAML::Key << "height" << YAML::Value << height;
        }
        virtual void readYaml(const YAML::Node& node) {
            node["width"] >> width;
            node["height"] >> height;
        }
    };

    State state;
    ConnectorIn* input_;
    QPixmap pixmap_;

    QGraphicsView* view_;

    QImage empty;
    QPainter painter;

    bool display_is_empty;

    bool down_;
    QPoint last_pos_;
};

}

#endif // OUTPUT_DISPLAY_H
