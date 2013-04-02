#ifndef DATABASE_IO_H
#define DATABASE_IO_H

/// COMPONENT
#include <db/database.h>

/**
 * @brief The DatabaseIO class is resposible for ex- and importing databases
 */
class DatabaseIO
{
public:
    DatabaseIO();

    /**
     * @brief save saves the database to the internal file
     * @return true, iff saving was successful
     */
    static bool save(const std::string& file, const Database* db);

    /**
     * @brief load loads the database from the internal file
     * @return true, iff loading was successful
     */
    static bool load(const std::string& file, Database *&db);

    /**
     * @brief loadConfig Import a config from a saved file
     * @param output Output: read config
     * @return true, iff a config could be read
     */
    static bool loadConfig(const std::string& file, Config& output);
};

#endif // DATABASE_IO_H
