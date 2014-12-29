#ifndef PARAMETER_FACTORY_H
#define PARAMETER_FACTORY_H

/// COMPONENT
#include <utils_param/parameter.h>
#include <utils_param/value_parameter.h>
#include <utils_param/set_parameter.h>
#include <utils_param/parameter_description.h>

namespace param
{

class ParameterFactory
{
public:
    /**
     * @brief makeEmpty creates an empty parameter pointer
     * @param type
     * @return
     */
    static Parameter::Ptr makeEmpty(const std::string& type);




    /**
     * @brief clone duplicates a parameter deeply
     * @param param
     * @return
     */
    static Parameter::Ptr clone(const Parameter *param);
    static Parameter::Ptr clone(const Parameter& param);
    static Parameter::Ptr clone(const Parameter::Ptr& param);






    /**
     * @brief declareRange
     * @param name
     * @param description
     * @param min
     * @param max
     * @param def
     * @param step
     * @return
     */
    template <typename T>
    static Parameter::Ptr declareRange(const std::string& name,
                                       const ParameterDescription& description,
                                       T min, T max, T def, T step);

    template <typename T>
    static Parameter::Ptr declareRange(const std::string& name, T min, T max, T def, T step)
    {
        return declareRange(name, ParameterDescription(), min, max, def, step);
    }




    /**
     * @brief declareInterval
     * @param name
     * @param description
     * @param min
     * @param max
     * @param def_min
     * @param def_max
     * @param step
     * @return
     */
    template <typename T>
    static Parameter::Ptr declareInterval(const std::string& name,
                                          const ParameterDescription& description,
                                          T min, T max, T def_min, T def_max, T step);

    template <typename T>
    static Parameter::Ptr declareInterval(const std::string& name, T min, T max, T def_min, T def_max, T step)
    {
        return declareInterval(name, ParameterDescription(), min, max, def_min, def_max, step);
    }




    /**
     * @brief declareBool
     * @param name
     * @param def
     * @return
     */
    static Parameter::Ptr declareBool(const std::string& name, bool def);
    static Parameter::Ptr declareBool(const std::string& name, const ParameterDescription& description, bool def);





    /**
     * @brief declareText
     * @param name
     * @param description
     * @param def
     * @return
     */
    static Parameter::Ptr declareText(const std::string& name, const ParameterDescription& description, const std::string& def);
    static Parameter::Ptr declareText(const std::string& name, const std::string& def);




    /**
     * @brief declarePath
     * @param name
     * @param description
     * @param is_file
     * @param def
     * @param filter
     * @param input
     * @param output
     * @return
     */
    static Parameter::Ptr declarePath(const std::string& name, const ParameterDescription& description,
                                      bool is_file, const std::string& def, const std::string& filter = "", bool input = false, bool output = false);




    /**
     * @brief declareFileInputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static Parameter::Ptr declareFileInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter = "");

    /**
     * @brief declareFileOutputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static Parameter::Ptr declareFileOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");

    /**
     * @brief declareFileInputOutputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static Parameter::Ptr declareFileInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");



    /**
     * @brief declareDirectoryInputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static Parameter::Ptr declareDirectoryInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter = "");


    /**
     * @brief declareDirectoryOutputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static Parameter::Ptr declareDirectoryOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");


    /**
     * @brief declareDirectoryInputOutputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static Parameter::Ptr declareDirectoryInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");





    /**
     * @brief declareTrigger
     * @param name
     * @param description
     * @return
     */
    static Parameter::Ptr declareTrigger(const std::string& name, const ParameterDescription& description);
    static Parameter::Ptr declareTrigger(const std::string& name);



    /**
     * @brief declareColorParameter
     * @param name
     * @param description
     * @param r
     * @param g
     * @param b
     * @return
     */
    static Parameter::Ptr declareColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b);
    static Parameter::Ptr declareColorParameter(const std::string& name, int r, int g, int b);






    /**
     * @brief declareParameterSet
     * @param name
     * @param description
     * @param set
     * @param def default value
     * @return
     */
    template <typename T>
    static Parameter::Ptr declareParameterSet(const std::string& name, const ParameterDescription& description,
                                              const std::map<std::string, T> & set_values,
                                              const T& def)
    {
        SetParameter::Ptr result(new SetParameter(name, description));
        result->setSet(set_values);
        if(!set_values.empty()) {
            result->def_ = def;
            result->set<T>(def);
        }

        return result;
    }
    template <typename T>
    static Parameter::Ptr declareParameterSet(const std::string& name,
                                              const std::map<std::string, T> & set,
                                              const T& def)
    {
        return declareParameterSet(name, ParameterDescription(), set, def);
    }



    /**
     * @brief declareParameterStringSet
     * @param name
     * @param description
     * @param set
     * @param def default value
     * @return
     */
    static Parameter::Ptr declareParameterStringSet(const std::string& name,
                                                    const ParameterDescription& description,
                                                    const std::vector<std::string> & set,
                                                    const std::string &def = "");
    static Parameter::Ptr declareParameterStringSet(const std::string& name,
                                                    const std::vector<std::string> & set,
                                                    const std::string &def = "");




    /**
     * @brief declareParameterBitSet
     * @param name
     * @param description
     * @param set
     * @param def
     * @return
     */
    static Parameter::Ptr declareParameterBitSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, int> &set, int def = 0);
    static Parameter::Ptr declareParameterBitSet(const std::string& name, const std::map<std::string, int> &set, int def = 0);


    /**
     * @brief declareParameterBitSet
     * @param name
     * @param description
     * @param set
     * @return
     */
    static Parameter::Ptr declareParameterBitSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, std::pair<int, bool> > &set);
    static Parameter::Ptr declareParameterBitSet(const std::string& name, const std::map<std::string, std::pair<int, bool> > &set);





    /**
     * @brief declareValue
     * @param name
     * @param description
     * @param def
     * @return
     */
    template <typename T>
    static Parameter::Ptr declareValue(const std::string& name, const ParameterDescription& description, const T& def)
    {
        ValueParameter::Ptr result(new ValueParameter(name, description));
        result->def_ = def;
        result->set<T>(def);

        return result;
    }
    template <typename T>
    static Parameter::Ptr declareValue(const std::string& name, const T& def)
    {
        return declareValue(name, ParameterDescription(), def);
    }



    /**
     * @brief declareOutputProgress
     * @param name
     * @param description
     * @return
     */
    static Parameter::Ptr declareOutputProgress(const std::string& name, const ParameterDescription& description = ParameterDescription(""));
};

}


#endif // PARAMETER_FACTORY_H
