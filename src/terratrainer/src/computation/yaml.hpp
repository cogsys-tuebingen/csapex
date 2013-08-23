#ifndef YAML_HPP
#define YAML_HPP
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>

namespace CMPYAML {

template<typename _Tp, typename _Tw>
inline void writeMatrix(const cv::Mat &mat, YAML::Emitter &emitter)
{
    for(int i = 0 ; i < mat.rows ; i++) {
        for(int j = 0 ; j < mat.cols ; j++) {
            _Tw value = (_Tw) mat.at<_Tp>(i,j);

            emitter << value;
        }
    }

}

template<typename _Tp, typename _Tw>
inline void writeDescriptor(const cv::Mat &desc, const int id, YAML::Emitter &emitter)
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "class" << YAML::Value << id;
    int step = desc.rows * desc.cols;
    emitter << YAML::Key << "descrStep" << YAML::Value << step;
    emitter << YAML::Key << "descr" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    writeMatrix<_Tp, _Tw>(desc, emitter);
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
}

template<typename _Tp, typename _Tw>
inline void writeDescriptorRows(const Mat &desc, const int id, YAML::Emitter &emitter)
{
    for(int j = 0 ; j < desc.rows ; j++) {
        cv::Mat roi(desc, cv::Rect(0,j,desc.cols,1));
        CMPYAML::writeDescriptor<_Tp, _Tw>(roi, id, emitter);
    }
}
}

#endif // YAML_HPP
