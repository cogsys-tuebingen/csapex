/// HEADER
#include <csapex/view/screenshot_dialog.h>

/// COMPONENT
#include <csapex/view/widget_controller.h>
#include <csapex/model/node_filter_proxy_model.h>
#include <csapex/utility/html_delegate.h>
#include <csapex/view/designer_view.h>
#include <csapex/model/graph_worker.h>

/// SYSTEM
#include <QLabel>
#include <QRadioButton>
#include <QButtonGroup>
#include <QFileDialog>
#include <QDialogButtonBox>

using namespace csapex;

ScreenshotDialog::ScreenshotDialog(GraphWorkerPtr graph, QWidget* widget, QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f), graph_(graph), widget_(widget)
{
    setWindowIcon(QIcon(":/image.png"));
    setWindowTitle("Make Screenshot");

    setModal(true);

    QVBoxLayout* layout = new QVBoxLayout;

    view_ = new QGraphicsView;
    layout->addWidget(view_);

    QButtonGroup* rect = new QButtonGroup;

    rect_full_ = new QRadioButton("Window");
    rect->addButton(rect_full_);
    rect_full_->setChecked(true);
    layout->addWidget(rect_full_);
    QObject::connect(rect_full_, SIGNAL(toggled(bool)), this, SLOT(refreshScreenshot()));

    rect_scene_ = new QRadioButton("Scene");
    rect->addButton(rect_scene_);
    layout->addWidget(rect_scene_);
    QObject::connect(rect_scene_, SIGNAL(toggled(bool)), this, SLOT(refreshScreenshot()));


    button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok
                                       | QDialogButtonBox::Reset
                                       | QDialogButtonBox::Cancel);

    connect(button_box_, SIGNAL(clicked(QAbstractButton*)), this, SLOT(handle(QAbstractButton*)));

    layout->addWidget(button_box_);

    setLayout(layout);
    refreshScreenshot();
}

void ScreenshotDialog::handle(QAbstractButton* button)
{
    switch(button_box_->buttonRole(button)) {
    case QDialogButtonBox::ButtonRole::ResetRole:
        refreshScreenshot();
        break;
    case QDialogButtonBox::ButtonRole::AcceptRole:
        save();
        break;
    case QDialogButtonBox::ButtonRole::RejectRole:
        reject();
        break;
    default:
        reject();
        break;
    }
}

void ScreenshotDialog::refreshScreenshot()
{
    QPixmap screenshot;

    if(rect_full_->isChecked()) {
        screenshot = widget_->grab();
    } else if(rect_scene_->isChecked()) {
        auto view = widget_->findChild<DesignerView*>();
        if(view) {
            screenshot = view->grab();
        }
    } else {
        Q_EMIT reject();
        return;
    }


    image_ = screenshot.toImage();

    delete view_->scene();
    view_->setScene(new QGraphicsScene);

    view_->scene()->addPixmap(screenshot);
    view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
}

void ScreenshotDialog::save()
{
    bool pause = graph_->isPaused();
    graph_->setPause(true);
    QString filename = QFileDialog::getSaveFileName(0, "Save Screenshot", "", "*.png");

    if(!filename.isEmpty()) {
        image_.save(filename);
    }
    graph_->setPause(pause);

    if(!filename.isEmpty()) {
        Q_EMIT accept();
    } else {
        Q_EMIT reject();
    }
}


/// MOC
#include "../../include/csapex/view/moc_screenshot_dialog.cpp"
