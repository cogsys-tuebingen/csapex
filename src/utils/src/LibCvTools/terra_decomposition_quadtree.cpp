#include "terra_decomposition_quadtree.h"

TerraQuadtreeDecomposition::TerraQuadtreeDecomposition(const cv::Mat &_image, const cv::Size &_min_region_size,
                                                       TerraDecomClassifierFeature *_classifier) :
    QuadtreeDecomposition(_image, _min_region_size, _classifier)
{
}

bool TerraQuadtreeDecomposition::iterate()
{
    TerraDecomClassifierFeature *terra_classifier = dynamic_cast<TerraDecomClassifierFeature*>(classifier_.get());

    if(quadtree_nodes_.size() == 0) {
         if(terra_classifier->classify(quadtree_root_))
            split_and_activate(quadtree_root_);
         else {
            quadtree_leaves_.push_back(&quadtree_root_);
         }
    } else {
        process_active_nodes();
    }

    if(debug_size_.width != -1 && debug_size_.height != -1)
        render_debug();

    return quadtree_nodes_.size() != 0;
}

void TerraQuadtreeDecomposition::regions(std::vector<cv_roi::TerraROI> &regions)
{
    for(CVQtNodesList::iterator it = quadtree_leaves_.begin() ; it != quadtree_leaves_.end() ; it++) {
        cv_roi::TerraROI tr;
        tr.roi.rect = *(*it);
        tr.id = classifications_[*it];
        regions.push_back(tr);
    }
}

void TerraQuadtreeDecomposition::process_active_nodes()
{
    TerraDecomClassifierFeature *terra_classifier = dynamic_cast<TerraDecomClassifierFeature*>(classifier_.get());

    CVQtNodesList list = quadtree_nodes_;
    quadtree_nodes_.clear();

    for(CVQtNodesList::iterator it = list.begin() ; it != list.end() ; it++){
        CVQt *node = *it;
        if(!min_size_reached(*node) && terra_classifier->classify(*node)) {
            split_and_activate(*node);
        } else {
            cv_roi::TerraID p;
            p.id    = terra_classifier->get_id();
            p.prob  = terra_classifier->get_prob();
            classifications_.insert(ClassificationEntry(node, p));
            quadtree_leaves_.push_back(node);
        }
    }
}

