#ifndef DIRECTORY_IO_H
#define DIRECTORY_IO_H

/// PROJECT
#include <common/global.hpp>
#include <data/frame.h>

/// SYSTEM
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

/**
 * @brief The DirectoryIO class is resposible for importing complete directories of frames
 */
class DirectoryIO
{
public:
    static int MAX_IMPORT_PER_DIR;

public:
    /**
     * @brief The AbortException class
     */
    class AbortException : public std::exception {};

public:
    /**
     * @brief DirectoryIO constructs an empty directory iterator
     */
    DirectoryIO();

public:
    /**
     * @brief import_raw call the callback function for all image files in the given directory
     * @param path the directory to iterate over
     * @param callback the function to call for each image
     */
    static DirectoryIO import_raw(const std::string& path, boost::function<bool(Frame::Ptr)> &  callback);

    /**
     * @brief import_cropped call the callback function for all sample directories in the given directory, using cropped samples
     * @param path the directory to iterate over
     * @param callback the function to call for each sample directories
     */
    static DirectoryIO import_cropped(const std::string& path, bool use_mask, boost::function<bool(Frame::Ptr)> &  callback);

    /**
     * @brief import_fullsized call the callback function for all sample directories in the given directory, using the complete samples
     * @param path the directory to iterate over
     * @param callback the function to call for each sample directories
     */
    static DirectoryIO import_fullsized(const std::string& path, bool use_mask, boost::function<bool(Frame::Ptr)> &  callback);

public:
    /**
     * @brief next Calls the callback on the next file
     * @return true, if a file was found
     */
    bool next();

    /**
     * @brief import_all imports the maximum number of files
     */
    void import_all();


private:
    DirectoryIO(const std::string& path, bool use_directories, boost::function<bool (const std::string&)> load_and_callback);

private:
    boost::filesystem::directory_iterator itr;
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end

    bool use_directories;
    boost::function<bool (const std::string&)> load_and_callback;
    int already_imported;
};

#endif // DIRECTORY_IO_H
