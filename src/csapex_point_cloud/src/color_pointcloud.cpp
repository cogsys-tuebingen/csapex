/// HEADER
#include "color_pointcloud.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

CSAPEX_REGISTER_CLASS(csapex::ColorPointCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ColorPointCloud::ColorPointCloud()
{
}

void ColorPointCloud::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<ColorPointCloud>(this), msg->value);
}

void ColorPointCloud::setup()
{
    setSynchronizedInputs(true);
    input_  = addInput<PointCloudMessage>("Labeled PointCloud");
    output_ = addOutput<PointCloudMessage>("Colored PointCloud");
}

namespace {
#define _HSV2RGB_(H, S, V, R, G, B) \
{ \
    double _h = H/60.; \
    int _hf = (int)floor(_h); \
    int _hi = ((int)_h)%6; \
    double _f = _h - _hf; \
    \
    double _p = V * (1. - S); \
    double _q = V * (1. - _f * S); \
    double _t = V * (1. - (1. - _f) * S); \
    \
    switch (_hi) \
{ \
    case 0: \
    R = 255.*V; G = 255.*_t; B = 255.*_p; \
    break; \
    case 1: \
    R = 255.*_q; G = 255.*V; B = 255.*_p; \
    break; \
    case 2: \
    R = 255.*_p; G = 255.*V; B = 255.*_t; \
    break; \
    case 3: \
    R = 255.*_p; G = 255.*_q; B = 255.*V; \
    break; \
    case 4: \
    R = 255.*_t; G = 255.*_p; B = 255.*V; \
    break; \
    case 5: \
    R = 255.*V; G = 255.*_p; B = 255.*_q; \
    break; \
} \
}
}

namespace implementation {

struct Color {
    Color(uchar _r = 0, uchar _g = 0, uchar _b = 0) :
        r(_r),
        g(_g),
        b(_b){}

    uchar r;
    uchar g;
    uchar b;
};

template<class PointT>
struct Impl {
    inline static void convert(const typename pcl::PointCloud<PointT>::Ptr src,
                               typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst)
    {
        dst->height = src->height;
        dst->header = src->header;
        dst->width  = src->width;

        std::map<unsigned int, Color> colors;
        colors.insert(std::make_pair(0, Color()));
        for(typename pcl::PointCloud<PointT>::const_iterator it = src->begin() ; it != src->end() ; ++it) {
            if(colors.find(it->label) == colors.end()) {
                double r,g,b;
                _HSV2RGB_((double) ((colors.size() * 77) % 360), 1.0, 1.0, r, g, b);
                colors.insert(std::make_pair(it->label,Color(r,g,b)));
            }
            Color c = colors.at(it->label);
            pcl::PointXYZRGB p(c.r, c.g, c.b);
            p.x = it->x;
            p.y = it->y;
            p.z = it->z;
            dst->push_back(p);
        }
    }
};

template<class PointT>
struct Conversion {
    static void apply(const typename pcl::PointCloud<PointT>::Ptr src,
                      typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst)
    {
        throw std::runtime_error("Type of pointcloud must be labeled!");
    }
};

template<>
struct Conversion<pcl::PointXYZL>{
    static void apply(const typename pcl::PointCloud<pcl::PointXYZL>::Ptr src,
                      typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst)
    {
      Impl<pcl::PointXYZL>::convert(src, dst);
    }

};

template<>
struct Conversion<pcl::PointXYZRGBL>{
    static void apply(const typename pcl::PointCloud<pcl::PointXYZRGBL>::Ptr src,
                        typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst)
    {
        Impl<pcl::PointXYZRGBL>::convert(src, dst);
    }
};
}

template <class PointT>
void ColorPointCloud::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    PointCloudMessage::Ptr out(new PointCloudMessage);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    implementation::Conversion<PointT>::apply(cloud, out_cloud);

    out->value = out_cloud;
    output_->publish(out);
}
