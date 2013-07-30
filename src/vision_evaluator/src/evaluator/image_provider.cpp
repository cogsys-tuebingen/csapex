/// HEADER
#include <vision_evaluator/image_provider.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

using namespace csapex;

std::map<std::string, ImageProvider::ProviderConstructor> ImageProvider::plugins;

ImageProvider::ImageProvider()
{
}

ImageProvider::~ImageProvider()
{;
}

bool ImageProvider::canHandle(const std::string& path,
                              boost::function<bool (ImageProvider*)> reference)
{
    bfs::path file(path);

    bool is_dir = bfs::is_directory(file) ;
    std::string ext = is_dir ? ".DIRECTORY" : file.extension().string();

    std::map<std::string, ProviderConstructor>::iterator handler = plugins.find(ext);

    if(handler == plugins.end()) {
        return false;
    }

    return reference(handler->second(""));
}


bool ImageProvider::canHandle(const std::string& path)
{
    bfs::path file(path);

    bool is_dir = bfs::is_directory(file) ;
    std::string ext = is_dir ? ".DIRECTORY" : file.extension().string();

    std::map<std::string, ProviderConstructor>::iterator handler = plugins.find(ext);

    return handler != plugins.end();
}

ImageProvider* ImageProvider::create(const std::string& path)
{
    bfs::path file(path);

    bool is_dir = bfs::is_directory(file) ;
    std::string ext = is_dir ? ".DIRECTORY" : file.extension().string();
    ProviderConstructor constructor = plugins[ext];

    std::cout << "extension is " << ext << std::endl;

    if(constructor.empty()) {
        return NULL;
    } else {
        ImageProvider* result = constructor(path);
        result->init();
        return result;
    }
}

void ImageProvider::init()
{
    doInit();
}

void ImageProvider::next()
{
    cv::Mat img, mask;

    next(img, mask);

    new_image(img, mask);
}

int ImageProvider::sleepTime()
{
    return 100;
}

void ImageProvider::enableBorder(bool border)
{

}


Memento::Ptr ImageProvider::getState() const
{
    return Memento::Ptr((Memento*) NULL);
}

void ImageProvider::setState(Memento::Ptr memento)
{

}
