#ifndef IMAGE_PROVIDER_H
#define IMAGE_PROVIDER_H

/// COMPONENT
#include "plugin.h"

/// SYSTEM
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
#include <map>
#include <opencv2/opencv.hpp>
#include <QFrame>
#include <string>
#include <vector>

#define REGISTER_IMAGE_PROVIDERS(type, ext, exts...) \
    namespace vision_evaluator { \
    class _____##type##ext##_registrator : public ImageProvider::Access {\
        static _____##type##ext##_registrator instance; \
        _____##type##ext##_registrator () {\
            register_provider(std::string(#ext) + ", " + #exts, boost::bind(type::createInstance, _1)); \
        } \
    };\
    _____##type##ext##_registrator _____##type##ext##_registrator::instance; \
    }

namespace vision_evaluator
{

class ImageProvider : public Plugin<ImageProvider>
{
    Q_OBJECT

public:
    static PluginPtr createMetaInstance() {
        assert(false);
    }
    virtual PluginPtr metaInstance() {
        assert(false);
    }

protected:
    ImageProvider(const std::string& label);

    typedef boost::function<ImageProvider*(const std::string&)> ProviderConstructor;

public:
    class Access
    {
    protected:
        void register_provider(const std::string& exts, ProviderConstructor constructor) {
            std::vector<std::string> strs;
            boost::split(strs, exts, boost::is_any_of(", "));

            for(std::vector<std::string>::const_iterator it = strs.begin(); it != strs.end(); ++it) {
                std::string ext = *it;

                if(ext.size() == 0) {
                    continue;
                }

                std::cout << "registering file extension \"" << ext << "\"" << std::endl;
                plugins["." + ext] = constructor;
            }
        }
    };

public:
    virtual ~ImageProvider();

public:
    virtual void insert(QBoxLayout* layout) {}

public:
    static ImageProvider* create(const std::string& path);
    static bool canHandle(const std::string& path);
    static bool canHandle(const std::string& path,
                          boost::function<bool(ImageProvider*)> reference);

    virtual void update_gui(QFrame* additional_holder) {}

    virtual bool hasNext() = 0;
    virtual void next() = 0;
    virtual int sleepTime();

Q_SIGNALS:
    void new_image(cv::Mat, cv::Mat);

private:
    static std::map<std::string, ProviderConstructor> plugins;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_H
