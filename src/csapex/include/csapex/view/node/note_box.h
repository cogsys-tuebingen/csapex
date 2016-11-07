#ifndef NOTE_BOX_H
#define NOTE_BOX_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/node/box.h>

class QTextEdit;

namespace csapex
{

class CSAPEX_QT_EXPORT NoteBox : public NodeBox
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    QString cssClass() {
        return QString("NoteBox");
    }


public:
    NoteBox(Settings& settings,
            NodeHandlePtr handle, NodeWorkerPtr worker,
            QIcon icon, GraphView* parent = 0);

    NoteBox(Settings& settings,
            NodeHandlePtr handle,
            QIcon icon, GraphView* parent = 0);

    ~NoteBox();

    void construct() override;
    void init() override;

    void setSelected(bool selected) override;

    void updateComponentInformation(Graph* graph) override;
    void updateThreadInformation() override;
    void updateFrequencyInformation() override;

protected:
    void paintEvent(QPaintEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;

    virtual void updateStylesheetColor() override;
    virtual void startResize() override;
    virtual void stopResize() override;

private:
    slim_signal::Connection connection_;

    QTextEdit* edit_;
};

}

#endif // NOTE_BOX_H
