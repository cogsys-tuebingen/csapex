#ifndef CV_GRID_HPP
#define CV_GRID_HPP
#include "grid.hpp"
#include "grid_attributes.hpp"

/**
 * @namespace Common operations for opencv when a grid is needed.
 */
namespace cv_grid {
/**
 * @brief Prepare a homogene grid using hls attributes for cell comparison.
 * @param grid      the reference the grid will be written to
 * @param img       the source image
 * @param rows      the amount of rows
 * @param cols      the amount of cols
 * @param eps       the mean error
 */
template<class Attribute, class Grid, class Params>
inline void  prepare_grid(Grid &grid, const cv::Mat &img, const int rows, const int cols, const Params &p, const cv::Mat &mask = cv::Mat(), const double thresh = 1.0)
{

    grid = Grid(rows,cols);

    int cell_height      = img.rows / rows;
    int cell_width       = img.cols / cols;
    int cell_height_rest = img.rows % rows;
    int cell_width_rest  = img.cols % cols;

    for(int i = 0 ; i < cols ; i++) {
        for(int j = 0 ; j < rows ; j++) {
            cv::Rect r(cell_width * i, cell_height * j, cell_width, cell_height);

            if(i == cols - 1)
                r.width  += cell_width_rest;
            if(j == rows - 1)
                r.height += cell_height_rest;


            if(!mask.empty()) {
                cv::Mat mask_roi(mask, r);
                cv::Scalar s = cv::sum(mask_roi);
                if(s[0] / (double)(r.width * r.height) < thresh) {
                    grid(i,j) = GridCell<cv::Rect, Attribute>(false);
                    continue;
                }
            }

            cv::Mat img_roi(img, r);
            Attribute attr = Attribute::generate(img_roi, p);
            grid(j,i) = GridCell<cv::Rect, Attribute>(r, attr);
        }
    }
}

typedef GridCell<cv::Rect, AttrScalar>          GridCellScalar;    /// a grid cell using hls attributes
typedef Grid_<GridCellScalar>                   GridScalar;        /// a grid based on hls grid cells
typedef GridCell<cv::Rect, AttrHistogram>       GridCellHist;
typedef Grid_<GridCellHist>                     GridHist;

template<class GridT>
inline void render_grid_count(const GridT &g1, const GridT &g2, cv::Mat &out, std::pair<int, int> &counts, int &valid)
{
    counts.first = 0;
    counts.second = 0;
    valid = 0;
    for(int i = 0 ; i < g1.cols() ; i++) {
        for(int j = 0 ; j < g1.rows() ; j++) {

            cv::Rect r = g1(j,i).bounding;
            if(!g1(j,i).enabled || !g2(j,i).enabled) {
                cv::rectangle(out, r, cv::Scalar(0, 255, 255), 1);
                continue;
            }

            bool cell_compare = g1(j,i) == g2(j,i);
            if(cell_compare) {
                cv::rectangle(out, r, cv::Scalar(0, 255, 0), 1);
                counts.first++;
            } else {
                cv::rectangle(out, r, cv::Scalar(0, 0, 255), 1);
                counts.second++;
            }
            valid++;
        }
    }

    if(out.rows < 42 && out.cols < 120)
        throw std::runtime_error("Image to small to render text!");
    std::stringstream text;
    text << "+" << counts.first << " | -" << counts.second << " | all: " << valid;
    cv::putText(out, text.str(), cv::Point(0, out.rows - 2), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 255), 2);

}

template<class GridT>
inline void render_grid(const GridT &g1, const GridT &g2, cv::Mat &out)
{
     std::pair<int,int> c(0,0);
     int v = 0;

     render_grid_count<GridT>(g1, g2, out, c, v);
}

}



#endif // CV_GRID_HPP
