/// HEADER
#include "image_provider.h"

/// SYSTEM
#include <boost/filesystem.hpp>
#include <QThread>
#include <QtConcurrentRun>

namespace bfs = boost::filesystem;

using namespace vision_evaluator;

std::map<std::string, ImageProvider::ProviderConstructor> ImageProvider::plugins;

ImageProvider::ImageProvider()
    : private_thread(NULL)
{
}

ImageProvider::~ImageProvider()
{
    if(private_thread) {
        private_thread->wait(1000);
        if(private_thread) {
            private_thread->terminate();
        }
        delete private_thread;
    }
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
    QtConcurrent::run(this, &ImageProvider::doInit);
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
