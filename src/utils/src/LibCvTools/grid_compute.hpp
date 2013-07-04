#ifndef CV_GRID_HPP
#define CV_GRID_HPP
#include "grid.hpp"
#include "grid_attributes.hpp"

/**
 * @namespace Common operations for opencv when a grid is needed.
 */
namespace cv_grid {
typedef datastructure::AttributedGridCell<cv::Rect, AttributeColorHLS>   GridCellHLS;    /// a grid cell using hls attributes
typedef datastructure::Grid<GridCellHLS>                            GridHLS;        /// a grid based on hls grid cells

/**
 * @brief Prepare a homogene grid using hls attributes for cell comparison.
 * @param grid      the reference the grid will be written to
 * @param img       the source image
 * @param rows      the amount of rows
 * @param cols      the amount of cols
 * @param eps       the mean error
 */
inline void  prepare_grid_hls(GridHLS &grid, const cv::Mat &img, const int rows, const int cols, cv::Scalar eps)
{
    grid = GridHLS(rows,cols);

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

            cv::Mat roi(img, r);

            AttributeColorHLS attr = AttributeColorHLS::generate(roi, eps);
            grid(j,i) = GridCellHLS(r, attr);

        }
    }
}

}



#endif // CV_GRID_HPP
