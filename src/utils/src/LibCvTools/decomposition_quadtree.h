#ifndef CV_QUADTREE_DECOMPOSITION_H
#define CV_QUADTREE_DECOMPOSITION_H
#include <opencv2/opencv.hpp>
#include "quad_tree.hpp"
#include "decomposition_classifiers.hpp"
/**
 * @brief   The QuadtreeDecomposition generates a quadtree base decomposition for images.
 *          Wether a region should be split into 4 new ones is determined by the given classifier.
 */
class QuadtreeDecomposition
{
    typedef Quadtree<cv::Vec2i, cv::Rect>                CVQt;              /// template init for quadtree
    typedef std::vector<CVQt*>                           CVQtNodesList;     /// a node list

public:
    /**
     * @brief QuadtreeDecomposition constructor.
     * @param image                 the image to decompose
     * @param _min_region_size      the minimal region size
     * @param _classifier           the classifier to be used
     */
    QuadtreeDecomposition(const cv::Mat &image, const cv::Size &_min_region_size, DecompositionClassifier &_classifier);

    /**
     * @brief Instant generation of the decomposition without single steps done.
     */
    void    auto_iterate();
    /**
     * @brief Do next computation step.
     * @return  if there are more steps to be done
     */
    bool    iterate();
    /**
     * @brief This method will spawn a window to observe the single steps in the decomposition.
     * @param window_size       this will be the debug output window size
     * @param debug_color       this will be the color for rendering regions
     */
    void    enable_debug_out_put(const cv::Size &window_size, const cv::Scalar &debug_color = cv::Scalar(255,255,255,255));
    /**
     * @brief Extract regions as rects.
     * @param regions       the vector the results will be written to
     */
    void    regions(std::vector<cv::Rect> &regions)
    {

        CVQtNodesList leaves;
        quadtree_root_.collect_leaves(leaves);
        for(CVQtNodesList::iterator it = leaves.begin() ; it != leaves.end() ; it++) {
            CVQt *node = *it;
            cv::Rect rect = *node;
            limit(rect, image_.cols, image_.rows);
            regions.push_back(rect);
        }

    }


    virtual ~QuadtreeDecomposition();
private:
    cv::Mat                 image_;
    cv::Size                min_region_size_;
    DecompositionClassifier    &classfier_;
    CVQt                    quadtree_root_;
    CVQtNodesList           quadtree_nodes_;

    bool                    auto_iterate_;
    cv::Size                debug_size_;
    cv::Scalar              debug_color_;

    void    process_active_nodes();
    void    render_debug();


    void limit(cv::Rect &rect, const int cols, const int rows)
    {
        rect.x = std::min(rect.x, cols - 1);
        rect.y = std::min(rect.y, rows - 1);
        rect.width  = std::min(rect.width,  cols - 1 - rect.x);
        rect.height = std::min(rect.height, rows - 1 - rect.y);
    }

    void split_and_activate(CVQt &node)
    {
        node.split();
        quadtree_nodes_.push_back(&node[0]);
        quadtree_nodes_.push_back(&node[1]);
        quadtree_nodes_.push_back(&node[2]);
        quadtree_nodes_.push_back(&node[3]);
    }

    bool min_size_reached(CVQt &node)
    {
        cv::Rect region(node.region());
        return region.width < min_region_size_.width || region.height < min_region_size_.height;
    }


};

#endif // CV_QUADTREE_DECOMPOSITION_H
