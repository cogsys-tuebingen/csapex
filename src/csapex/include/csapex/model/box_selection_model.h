//#ifndef BOX_SELECTION_MODEL_H
//#define BOX_SELECTION_MODEL_H

///// COMPONENT
//#include <csapex/model/selection_model.h>

///// SYSTEM
//#include <QMenu>
//#include <boost/function.hpp>

//namespace csapex
//{
//class BoxSelectionModel : public SelectionModel
//{
//    Q_OBJECT

//public:
//    BoxSelectionModel(GraphPtr graph, WidgetController* widget_ctrl);

//    void handleSelection(Node* node, bool add);
//    CommandPtr deleteSelectedCommand();

//    void select(Node* node, bool add = false);
//    int countSelected();

//    void fillContextMenuForSelection(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);


//public Q_SLOTS:
//    void clearSelection();
//    void selectAll();

//    void toggleSelection(NodeBox* box);
//    void mimicBoxMovement(NodeBox* box, int dx, int dy);

//    void moveSelectedBoxes(NodeBox* origin, const QPoint& delta);
//};
//}

//#endif // BOX_SELECTION_MODEL_H
