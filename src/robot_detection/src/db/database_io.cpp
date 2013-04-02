/// HEADER
#include "database_io.h"

/// COMPONENT
#include "serializer.h"

/// PROJECT
#include <config/config.h>
#include <common/global.hpp>

DatabaseIO::DatabaseIO()
{
}


bool DatabaseIO::save(const std::string& file, const Database* db)
{
    INFO("save to " << file);

    std::ofstream ofs(file.c_str());
    if(ofs.is_open()) {
//        boost::iostreams::filtering_ostreambuf out_filter_stream;
//        out_filter_stream.push(boost::iostreams::zlib_compressor(boost::iostreams::zlib::best_compression));
//        out_filter_stream.push(ofs);
//        boost::archive::binary_oarchive archive(out_filter_stream);
        boost::archive::text_oarchive archive(ofs);

        Config cfg = Config::getGlobal();

        try {
            archive << cfg;
            archive << db;

            db->replaced();

        } catch(boost::archive::archive_exception& e) {
            ERROR("cannot save the database at " << file);
            return false;
        }

        INFO("success");
        return true;

    } else {
        ERROR("the file " << file << " does not exist or is not writeable");
        return false;
    }
}

bool DatabaseIO::load(const std::string& file, Database*& db)
{
    std::ifstream ifs(file.c_str());
    if(ifs.is_open()) {
//        boost::iostreams::filtering_istreambuf in_filter_stream;
//        in_filter_stream.push(boost::iostreams::zlib_decompressor());
//        in_filter_stream.push(ifs);
//        boost::archive::binary_iarchive archive(in_filter_stream);

        boost::archive::text_iarchive archive(ifs);

        Config cfg = Config::getGlobal();

        try {
            // load config data but ignore them
            archive >> cfg;

            Database* db_tmp;
            archive >> db_tmp;

            db = db_tmp;

            db_tmp->replaced();

        } catch(boost::archive::archive_exception& e) {
            ERROR("cannot load the database at " << file);
            return false;
        }


        return true;

    } else {
        ERROR("the file " << file << " does not exist or is not readable");
        return false;
    }
}


bool DatabaseIO::loadConfig(const std::string& file, Config& cfg)
{
    std::ifstream ifs(file.c_str());
    if(ifs.is_open()) {
//        boost::iostreams::filtering_istreambuf in_filter_stream;
//        in_filter_stream.push(boost::iostreams::zlib_decompressor());
//        in_filter_stream.push(ifs);
//        boost::archive::binary_iarchive archive(in_filter_stream);

        boost::archive::text_iarchive archive(ifs);

        try {
            archive >> cfg;

        } catch(boost::archive::archive_exception& e) {
            ERROR("cannot load the database at " << file);
            return false;
        }

        return true;

    } else {
        ERROR("the file " << file << " does not exist or is not readable");
        return false;
    }
}
