#ifndef NOTE_BOX_H
#define NOTE_BOX_H

/// COMPONENT
#include <csapex_qt/export.h>
#include <csapex/view/node/box.h>

class QTextEdit;

namespace csapex
{
class CSAPEX_QT_EXPORT StickyNoteBox : public NodeBox
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    QString cssClass()
    {
        return QString("NoteBox");
    }

public:
    StickyNoteBox(Settings& settings, NodeFacadePtr node_facade_, QIcon icon, GraphView* parent = 0);

    ~StickyNoteBox() override;

    void construct() override;
    void init() override;

    void setSelected(bool selected) override;

    void updateComponentInformation(GraphFacade* graph) override;
    void updateThreadInformation() override;
    void updateFrequencyInformation() override;

protected:
    void paintEvent(QPaintEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;

    void refreshTopLevelStylesheet() override;
    void updateStylesheetColor() override;
    void startResize() override;
    void stopResize() override;

private:
    slim_signal::Connection connection_;

    QObserver observer_;

    QColor default_bg_color_;
    const int border_color_difference_;

    QTextEdit* edit_;
};

}  // namespace csapex

#endif  // NOTE_BOX_H
