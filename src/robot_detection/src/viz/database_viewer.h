#ifndef DATABASE_VIEWER_H
#define DATABASE_VIEWER_H

/// COMPONENT
#include "database_io_window.h"

/// PROJECT
#include <db_strategy/db_strategy.h>

/// SYSTEM
#include <QGraphicsScene>
#include <QSlider>

/// FORWARD DECLARATION
class MatchablePose;

namespace Ui
{
class DatabaseViewer;
}

/**
 * @brief The DatabaseViewer class renders the robot at a selectable orientation
 */
class DatabaseViewer : public DatabaseIOWindow
{
    Q_OBJECT

public:
    /**
     * @brief DatabaseViewer
     * @param db_file the database to use
     * @param parent
     */
    explicit DatabaseViewer(QWidget* parent = 0);

    /**
     * @brief ~DatabaseViewer
     */
    virtual ~DatabaseViewer();

public Q_SLOTS:
    void renderAngle(int angle);
    void renderSelection();
    void deleteSelection();

private:
    void renderPose(MatchablePose* p);

private:
    Ui::DatabaseViewer* ui;

    DatabaseStrategyInterface::Ptr db;

    QGraphicsScene* current_scene;
};

#endif // DATABASE_VIEWER_H
