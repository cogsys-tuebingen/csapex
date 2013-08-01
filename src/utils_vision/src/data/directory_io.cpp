/// HEADER
#include "directory_io.h"

/// COMPONENT
#include "frame_io.h"

namespace bfs = boost::filesystem;

int DirectoryIO::MAX_IMPORT_PER_DIR = 0;

DirectoryIO::DirectoryIO(const std::string& path, bool use_directories, boost::function<bool (const std::string&)> load_and_callback)
    : itr(path), use_directories(use_directories), load_and_callback(load_and_callback), already_imported(0)
{
    INFO("importing <=" << DirectoryIO::MAX_IMPORT_PER_DIR << " from " << path);
}

DirectoryIO::DirectoryIO()
    : use_directories(false), already_imported(0)
{
    itr = end_itr;
}

DirectoryIO DirectoryIO::import_raw(const std::string& path, boost::function<bool (Frame::Ptr)> &callback)
{
    return DirectoryIO(path, false, boost::bind(callback, boost::bind(FrameIO::importRaw, _1)));
}

DirectoryIO DirectoryIO::import_cropped(const std::string& path, bool use_mask, boost::function<bool (Frame::Ptr)> &callback)
{
    return DirectoryIO(path, true, boost::bind(callback, boost::bind(FrameIO::importCropped, _1, use_mask)));
}

DirectoryIO DirectoryIO::import_fullsized(const std::string& path, bool use_mask, boost::function<bool (Frame::Ptr)> &callback)
{
    return DirectoryIO(path, true, boost::bind(callback, boost::bind(FrameIO::importFullsize, _1, use_mask)));
}

bool DirectoryIO::next()
{
    bool everything_imported = (DirectoryIO::MAX_IMPORT_PER_DIR > 0) && (already_imported >= DirectoryIO::MAX_IMPORT_PER_DIR);
    if(itr == end_itr || everything_imported) {
        return false;
    }

    if(bfs::is_directory(itr->status()) == use_directories) {
        bool stop = !load_and_callback(std::string(itr->path().string()));
        ++already_imported;

        if(stop) {
            throw AbortException();
        }
    }

    itr++;

    return true;
}

void DirectoryIO::import_all()
{
    try {
        while(next());
    } catch(DirectoryIO::AbortException& e) {
        INFO("aborted iteration");
    }
}
