#ifndef DESIGNER_H
#define DESIGNER_H

/// SYSTEM
#include <QWidget>

/// FORWARD DECLARATIONS
namespace Ui
{
class Designer;
}

namespace csapex
{

class Box;

class Designer : public QWidget
{
    Q_OBJECT

    friend class DesignerIO;

public:
    Designer(QWidget* parent = 0);
    virtual ~Designer();

    bool eventFilter(QObject* o, QEvent* e);
    void resizeEvent(QResizeEvent* e);

    bool isDirty();

    bool canUndo();
    bool canRedo();

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);

    void updateCursor();

    std::string getConfig();

public Q_SLOTS:
    void save();
    void saveAs();
    void load();
    void reload();
    void undo();
    void redo();
    void clear();
    void setCurrentConfig(const std::string& filename);


Q_SIGNALS:
    void stateChanged();
    void configChanged();

private:
    Ui::Designer* ui;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;

    std::string current_config_;
};

}
#endif // DESIGNER_H
