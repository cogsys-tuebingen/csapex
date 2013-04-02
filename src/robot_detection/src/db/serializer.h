#ifndef SERIALIZER_H
#define SERIALIZER_H

/// SYSTEM
#include <boost/archive/polymorphic_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

/// EXPORT
#include <boost/serialization/export.hpp>
#include <db/bin_database.h>
#include <db/bag_database.h>
#include <db/bow_database.h>
#include <db/naive_database.h>

BOOST_CLASS_EXPORT(Database)
BOOST_CLASS_EXPORT(BinDatabase)
BOOST_CLASS_EXPORT(BagDatabase)
BOOST_CLASS_EXPORT(BowDatabase)
BOOST_CLASS_EXPORT(NaiveDatabase)
//BOOST_CLASS_EXPORT_GUID(BinDatabase, "BinDatabase")
//BOOST_CLASS_EXPORT_GUID(BowDatabase, "BowDatabase")
//BOOST_CLASS_EXPORT_GUID(NaiveDatabase, "NaiveDatabase")

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
BOOST_SERIALIZATION_SPLIT_FREE(cv::KeyPoint)
BOOST_SERIALIZATION_SPLIT_FREE(cv::Point_<float>)

namespace boost
{
namespace serialization
{

/*** Mat ***/
template<class Archive>
void save(Archive& ar, const cv::Mat& m, const unsigned int version)
{
    size_t elemSize = m.elemSize(), elemType = m.type();

    ar& m.cols;
    ar& m.rows;
    ar& elemSize;
    ar& elemType;  // element type.
    size_t dataSize = m.cols * m.rows * m.elemSize();

    for(size_t dc = 0; dc < dataSize; ++dc) {
        ar& m.data[dc];
    }
}

template<class Archive>
void load(Archive& ar, cv::Mat& m, const unsigned int version)
{
    int cols, rows;
    size_t elemSize, elemType;

    ar& cols;
    ar& rows;
    ar& elemSize;
    ar& elemType;

    m.create(rows, cols, elemType);
    size_t dataSize = m.cols * m.rows * elemSize;

    for(size_t dc = 0; dc < dataSize; ++dc) {
        ar& m.data[dc];
    }
}


/*** Keypoint ***/
template<class Archive>
void save(Archive& ar, const cv::KeyPoint& k, const unsigned int version)
{
    ar& k.angle;
    ar& k.class_id;
    ar& k.octave;
    ar& k.pt;
    ar& k.response;
    ar& k.size;
}

template<class Archive>
void load(Archive& ar, cv::KeyPoint& k, const unsigned int version)
{
    ar& k.angle;
    ar& k.class_id;
    ar& k.octave;
    ar& k.pt;
    ar& k.response;
    ar& k.size;
}


/*** Point ***/
template<class Archive>
void save(Archive& ar, const cv::Point2f& p, const unsigned int version)
{
    ar& p.x;
    ar& p.y;
}

template<class Archive>
void load(Archive& ar, cv::Point2f& p, const unsigned int version)
{
    ar& p.x;
    ar& p.y;
}

}
}

#endif // SERIALIZER_H
