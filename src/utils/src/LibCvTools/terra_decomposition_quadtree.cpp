#include "terra_decomposition_quadtree.h"

TerraQuadtreeDecomposition::TerraQuadtreeDecomposition(const cv::Mat &_image, const cv::Size &_min_region_size,
                                                       TerraDecomClassifier &_classifier) :
    QuadtreeDecomposition(_image, _min_region_size, _classifier)
{
}

bool TerraQuadtreeDecomposition::iterate()
{
    TerraDecomClassifier       &terra_classifier_ = *((TerraDecomClassifier *) &classifier_);

    if(quadtree_nodes_.size() == 0) {
         if(classifier_.classify(quadtree_root_))
            split_and_activate(quadtree_root_);
         else {

         }
    } else {
        process_active_nodes();
    }

    if(debug_size_.width != -1 && debug_size_.height != -1)
        render_debug();

    return quadtree_nodes_.size() != 0;
}

void TerraQuadtreeDecomposition::regions(std::vector<TerraRect> &regions)
{
    for(CVQtNodesList::iterator it = leaves_.begin() ; it != leaves_.end() ; it++) {
        TerraRect tr;
        tr.rect = *(*it);
        tr.id = classifications_[*it];
        regions.push_back(tr);
    }
}

void TerraQuadtreeDecomposition::process_active_nodes()
{
    TerraDecomClassifier       &terra_classifier_ = *((TerraDecomClassifier *) &classifier_);

    CVQtNodesList list = quadtree_nodes_;
    quadtree_nodes_.clear();

    for(CVQtNodesList::iterator it = list.begin() ; it != list.end() ; it++){
        CVQt *node = *it;
        if(!min_size_reached(*node) && terra_classifier_.classify(*node)) {
            split_and_activate(*node);
        } else {
            TerraID p;
            p.id    = terra_classifier_.get_id();
            p.prob  = terra_classifier_.get_prob();
            classifications_.insert(ClassificationEntry(node, p));
        }
    }
}

