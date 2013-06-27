#ifndef QUAD_TREE_HPP
#define QUAD_TREE_HPP
#include <vector>
#include <iostream>
#include <cmath>

/**
 * @class Represenation of a quad tree, given regions with origins.
 */
template<class Point, class Rect>
class Quadtree{
public:
    /**
     * @brief Quadtree default constructor.
     */
    Quadtree()
    {
    }

    /**
     * @brief Quadtree constructor.
     * @param _region   the region the quadtree will represent
     */
    Quadtree(const Rect &_region) :
        region_(_region)
    {
    }

    /**
     * @brief Quadtree copy constructor.
     * @param q     the quadtree to copy
     */
    Quadtree(const Quadtree &q) :
        region_(q.region_),
        sub_regions_(q.sub_regions_)
    {
    }

    /**
     * @brief Quadtree destructor.
     */
    virtual ~Quadtree()
    {
    }

    /**
     * @brief Splits the region into 4 subregions.
     *        Alignment Policy:
     *        - south west: x aligned to parent region
     *        - north west: x aligned to parent region
     */
    void split()
    {
        sub_regions_.resize(4);

        Point widths;
        widths[0] = std::floor(region_.width / 2.0);
        widths[1] = std::ceil(region_.width / 2.0);
        Point heights;
        heights[0] = std::floor(region_.height /2.0);
        heights[1] = std::ceil(region_.height / 2.0);
        Point middle(region_.x, region_.y);
        middle[0] += widths[0];
        middle[1] += heights[0];

        Rect nw(region_.x, region_.y,widths[0], heights[0]);
        Rect sw(region_.x, middle[1], widths[0], heights[1]);
        Rect ne(middle[0], region_.y,widths[1], heights[0]);
        Rect se(middle[0], middle[1], widths[1], heights[1]);

        sub_regions_[0] = Quadtree(nw);
        sub_regions_[1] = Quadtree(sw);
        sub_regions_[2] = Quadtree(ne);
        sub_regions_[3] = Quadtree(se);

    }

    /**
     * @brief Access the regions given by id
     * @param i     the region id {nw = 0, sw = 1, ne = 2, se = 3}
     * @return      the region selected
     */
    Quadtree& operator[](const int i)
    {
        return sub_regions_[i];
    }

    /**
     * @brief Return the north west region.
     * @return      the north west region
     */
    Quadtree& north_west()
    {
        return sub_regions_[0];
    }

    /**
     * @brief Return the southwest region.
     * @return      the south west region
     */
    Quadtree& south_west()
    {
        return sub_regions_[1];
    }

    /**
     * @brief Return the north east region.
     * @return      the north east region
     */
    Quadtree& north_east()
    {
        return sub_regions_[2];
    }

    /**
     * @brief Return the south east region.
     * @return      the south east region
     */
    Quadtree& south_east()
    {
        return sub_regions_[3];
    }

    /**
     * @brief Casting the tree to a rectangle will return the bounding rectangle.
     */
    operator Rect()
    {
        return region_;
    }

    /**
     * @brief Return the region.
     * @return      the bounding region
     */
    Rect region()
    {
        return region_;
    }

    /**
     * @brief Check if this node is already a leaf and does not have any children.
     * @return
     */
    const bool is_leaf()
    {
        return sub_regions_.size() == 0;
    }

    /**
     * @brief Collect all
     * @param leaves
     */
    const void collect_leaves(std::vector<Quadtree<Point, Rect>* > &leaves)
    {
        if(is_leaf())
            leaves.push_back(this);
        else {
            sub_regions_[0].collect_leaves(leaves);
            sub_regions_[1].collect_leaves(leaves);
            sub_regions_[2].collect_leaves(leaves);
            sub_regions_[3].collect_leaves(leaves);
        }
    }

private:
    Rect    region_;
    std::vector<Quadtree<Point, Rect> > sub_regions_;
};

#endif // QUAD_TREE_HPP
