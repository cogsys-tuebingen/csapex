#ifndef GRID_HPP
#define GRID_HPP
#include <vector>
#include <assert.h>

/**
 * @class The AttributedGridCell class is used as the content for a cell of a grid.
 *        An attribute is a value that can be used to compare two cells.
 */
template<class Rect, class Attributes>
class AttributedGridCell {
public:
    /**
     * @brief AttributedGridCell default constructor.
     */
    AttributedGridCell()
    {
    }

    /**
     * @brief AttributedGridCell constructor.
     * @param _bounding         the grid cells bounding box
     * @param _attributes       the grid cells attributes
     */
    AttributedGridCell(const Rect _bounding, const Attributes _attributes) :
        bounding(_bounding),
        attributes(_attributes)
    {
    }

    /**
     * @brief AttributedGridCell copy constructor.
     * @param c     another grid cell
     */
    AttributedGridCell(const AttributedGridCell<Rect, Attributes> &c) :
        bounding(c.bounding),
        attributes(c.attributes)
    {
    }

    /**
     * @brief ~AttributedGridCell default destructor.
     */
    virtual ~AttributedGridCell()
    {
    }

    /**
     *  Calculate the grid cells center and retrun it.
     *  @param p    the reference the result will be written to
     */
    template<class Point>
    void get_center(Point &p)
    {
        p.x = bounding.x + bounding.width / 2.0;
        p.y = bounding.y + bounding.height / 2.0;
    }

    /**
     * @brief Compare two grid cells
     * @param g     another grid cell
     * @return      if the cells are equal in a defined way
     */
    bool operator == (const AttributedGridCell<Rect, Attributes> g) const
    {
        return g.attributes == attributes;
    }

    Rect        bounding;           /// bounding box of the grid
    Attributes  attributes;         /// the attributes to compare
};

/**
 * @class   The Grid class can be used to make the usage of a Grid easier.
 *          It can be observed as a functional wrapper class.
 */
template<class Cell>
class Grid {
public:
    /**
     * @brief Grid default constructor.
     */
    Grid() :
        rows_(0),
        cols_(0),
        cells_(0)
    {
    }

    /**
     * @brief Grid constructor.
     * @param rows      the amount of rows
     * @param cols      the amount of cols
     */
    Grid(const int rows, const int cols) :
        rows_(rows),
        cols_(cols),
        cells_(rows * cols)
    {
    }

    /**
     * @brief Grid copy constructor.
     * @param g         another grid
     */
    Grid(const Grid& g) :
        rows_(g.rows_),
        cols_(g.cols_),
        cells_(g.rows_ * g.cols_)
    {
        memcpy(cells_.data(), g.cells_.data(), sizeof(Cell) * cols_ * rows_);
    }

    /**
     * @brief ~Grid default destructor.
     */
    virtual ~Grid()
    {
    }

    /**
     * @brief Access a cell.
     * @param row       row index
     * @param cols      col index
     * @return          a reference to the cells
     */
    Cell & operator() (const unsigned int row, const unsigned int col)
    {
        assert(col < cols_);
        assert(row < rows_);

        return cells_[col + row * cols_];
    }

    /**
     * @brief Access a cell.
     * @param row       row index
     * @param cols      col index
     * @return          a const reference
     */
    Cell & operator() (const unsigned int row, const unsigned int col) const
    {
        assert(col < cols_);
        assert(row < rows_);

        return const_cast<Cell&> (cells_[col + row * cols_]);
    }

    /**
     * @brief Return amount of rows.
     * @return          the amount of rows
     */
    int rows() const
    {
        return rows_;
    }

    /**
     * @brief Return amount of cols.
     * @return          the amount of cols
     */
    int cols() const
    {
        return cols_;
    }

    /**
     * @brief Compare to grids.
     * @param g         another grid
     * @return          if the grids are equal
     */
    bool operator == (const Grid<Cell> &g)
    {
        assert(cols_ == g.cols_);
        assert(rows_ == g.rows_);

        bool res = true;
        for(int i = 0 ; i < cols_ ; i++) {
            for(int j = 0 ; j < rows_ ; j++) {
                res &= g.cells_[i + j * cols_] == cells_[i + j * cols_];
            }
        }

        return res;
    }

private:
int                     rows_;      /// amount of rows
int                     cols_;      /// amount of cols
std::vector<Cell>       cells_;     /// the cells
};
#endif // GRID_HPP
