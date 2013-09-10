#ifndef IMAGE_PROVIDER_H
#define IMAGE_PROVIDER_H

/// PROJECT
#include <csapex/model/memento.h>
#include <csapex/model/message_provider.h>

/// SYSTEM
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
#include <map>
#include <opencv2/opencv.hpp>
#include <QBoxLayout>
#include <QFrame>
#include <string>
#include <vector>


namespace csapex
{

class ImageProvider : public MessageProvider
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<ImageProvider> Ptr;

protected:
    typedef boost::function<ImageProvider*(const std::string&)> ProviderConstructor;

public:
    ImageProvider();
    virtual ~ImageProvider();

public:
    virtual void update_gui(QFrame* additional_holder) {}
    virtual connection_types::Message::Ptr next();

    std::vector<std::string> getExtensions() const;

public:
    static ImageProvider* create(const std::string& path);
    static bool canHandle(const std::string& path);
    static bool canHandle(const std::string& path,
                          boost::function<bool(ImageProvider*)> reference);


    void init();
    virtual void doInit() {}
    virtual bool hasNext() = 0;
    virtual void next(cv::Mat&, cv::Mat&) = 0;
    virtual int sleepTime();

    virtual void enableBorder(bool border);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

private:
    static std::map<std::string, ProviderConstructor> plugins;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_H
