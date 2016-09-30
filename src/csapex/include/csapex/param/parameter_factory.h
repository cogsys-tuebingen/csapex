#ifndef PARAMETER_FACTORY_H
#define PARAMETER_FACTORY_H

/// COMPONENT
#include <csapex/param/parameter.h>
#include <csapex/param/parameter_builder.h>
#include <csapex/param/parameter_description.h>
#include <csapex/csapex_param_export.h>

namespace csapex
{
namespace param
{

class CSAPEX_PARAM_EXPORT ParameterFactory
{
public:
    /**
     * @brief makeEmpty creates an empty parameter pointer
     * @param type
     * @return
     */
    static ParameterBuilder makeEmpty(const std::string& type);




    /**
     * @brief clone duplicates a parameter deeply
     * @param param
     * @return
     */
    static ParameterBuilder clone(const Parameter *param);
    static ParameterBuilder clone(const Parameter& param);
    static ParameterBuilder clone(const std::shared_ptr<Parameter>& param);






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
    static ParameterBuilder declareRange(const std::string& name,
                                       const ParameterDescription& description,
                                       T min, T max, T def, T step);

    template <typename T>
    static ParameterBuilder declareRange(const std::string& name, T min, T max, T def, T step)
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
    static ParameterBuilder declareInterval(const std::string& name,
                                          const ParameterDescription& description,
                                          T min, T max, T def_min, T def_max, T step);

    template <typename T>
    static ParameterBuilder declareInterval(const std::string& name, T min, T max, T def_min, T def_max, T step)
    {
        return declareInterval(name, ParameterDescription(), min, max, def_min, def_max, step);
    }




    /**
     * @brief declareBool
     * @param name
     * @param def
     * @return
     */
    static ParameterBuilder declareBool(const std::string& name, bool def);
    static ParameterBuilder declareBool(const std::string& name, const ParameterDescription& description, bool def);





    /**
     * @brief declareText
     * @param name
     * @param description
     * @param def
     * @return
     */
    static ParameterBuilder declareText(const std::string& name, const ParameterDescription& description, const std::string& def);
    static ParameterBuilder declareText(const std::string& name, const std::string& def);




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
    static ParameterBuilder declarePath(const std::string& name, const ParameterDescription& description,
                                      bool is_file, const std::string& def, const std::string& filter = "", bool input = false, bool output = false);




    /**
     * @brief declareFileInputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static ParameterBuilder declareFileInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static ParameterBuilder declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter = "");

    /**
     * @brief declareFileOutputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static ParameterBuilder declareFileOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static ParameterBuilder declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");

    /**
     * @brief declareFileInputOutputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static ParameterBuilder declareFileInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static ParameterBuilder declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");



    /**
     * @brief declareDirectoryInputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static ParameterBuilder declareDirectoryInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static ParameterBuilder declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter = "");


    /**
     * @brief declareDirectoryOutputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static ParameterBuilder declareDirectoryOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static ParameterBuilder declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");


    /**
     * @brief declareDirectoryInputOutputPath
     * @param name
     * @param description
     * @param def
     * @param filter
     * @return
     */
    static ParameterBuilder declareDirectoryInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
    static ParameterBuilder declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");





    /**
     * @brief declareTrigger
     * @param name
     * @param description
     * @return
     */
    static ParameterBuilder declareTrigger(const std::string& name, const ParameterDescription& description);
    static ParameterBuilder declareTrigger(const std::string& name);



    /**
     * @brief declareAngleParameter
     * @param name
     * @param description
     * @param angle default value
     * @return
     */
    static ParameterBuilder declareAngle(const std::string& name, const ParameterDescription& description, double angle);
    static ParameterBuilder declareAngle(const std::string& name, double angle);



    /**
     * @brief declareColorParameter
     * @param name
     * @param description
     * @param r
     * @param g
     * @param b
     * @return
     */
    static ParameterBuilder declareColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b);
    static ParameterBuilder declareColorParameter(const std::string& name, int r, int g, int b);






    /**
     * @brief declareParameterSet
     * @param name
     * @param description
     * @param set
     * @param def default value
     * @return
     */
    template <typename T>
    static ParameterBuilder declareParameterSet(const std::string& name, const ParameterDescription& description,
                                              const std::map<std::string, T> & set_values,
                                              const T& def);
    template <typename T>
    static ParameterBuilder declareParameterSet(const std::string& name,
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
    static ParameterBuilder declareParameterStringSet(const std::string& name,
                                                    const ParameterDescription& description,
                                                    const std::vector<std::string> & set,
                                                    const std::string &def = "");
    static ParameterBuilder declareParameterStringSet(const std::string& name,
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
    static ParameterBuilder declareParameterBitSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, int> &set, int def = 0);
    static ParameterBuilder declareParameterBitSet(const std::string& name, const std::map<std::string, int> &set, int def = 0);


    /**
     * @brief declareParameterBitSet
     * @param name
     * @param description
     * @param set
     * @return
     */
    static ParameterBuilder declareParameterBitSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, std::pair<int, bool> > &set);
    static ParameterBuilder declareParameterBitSet(const std::string& name, const std::map<std::string, std::pair<int, bool> > &set);





    /**
     * @brief declareValue
     * @param name
     * @param description
     * @param def
     * @return
     */
    template <typename T>
    static ParameterBuilder declareValue(const std::string& name, const ParameterDescription& description, const T& def);

    template <typename T>
    static ParameterBuilder declareValue(const std::string& name, const T& def)
    {
        return declareValue(name, ParameterDescription(), def);
    }



    /**
     * @brief declareOutputProgress
     * @param name
     * @param description
     * @return
     */
    static ParameterBuilder declareOutputProgress(const std::string& name, const ParameterDescription& description = ParameterDescription(""));

    /**
     * @brief declareOutputText
     * @param name
     * @param description
     * @return
     */
    static ParameterBuilder declareOutputText(const std::string& name, const ParameterDescription& description = ParameterDescription(""));
};

}
}

#endif // PARAMETER_FACTORY_H
