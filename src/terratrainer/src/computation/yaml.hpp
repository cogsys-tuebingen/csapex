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

template<class T>
void readSequence(const YAML::Iterator &begin, const YAML::Iterator &end,
                  std::vector<T> &buffer)
{
    for(YAML::Iterator it = begin ; it != end ; it++) {
        T value;
        (*it) >> value;
        buffer.push_back(value);
    }
}

template<class T>
void writeSequence(YAML::Emitter &emitter, const std::vector<T> &buffer)
{
    emitter << YAML::BeginSeq;
    for(typename std::vector<T>::const_iterator it = buffer.begin() ; it != buffer.end() ; it++)
        emitter << *it;
    emitter << YAML::EndSeq;
}
}

#endif // YAML_HPP
