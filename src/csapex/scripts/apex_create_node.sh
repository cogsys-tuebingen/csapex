#!/bin/bash

if [[ $(grep "::" <<< $1) ]]; then
    arrIN=(${1//::/ })
    NODE_NAME=${arrIN[1]}
    NAMESPACE="${arrIN[0]}::"
    NAMESPACE_BEGIN="namespace ${arrIN[0]}{"
    NAMESPACE_END="}"
    NAMESPACE_USE="using namespace ${arrIN[0]};"
else
    NODE_NAME=$1
    NAMESPACE=""
    NAMESPACE_BEGIN=""
    NAMESPACE_END=""
    NAMESPACE_USE=""
fi

FILE_NAME=$(sed -e 's/\([A-Z]\)/_\L\1/g' -e 's/^_//' <<< $NODE_NAME)
FILE_NAME=$(tr '[:upper:]' '[:lower:]' <<< $FILE_NAME)


DESCRIPTION=$2

if [[ ! NODE_NAME || ! $DESCRIPTION ]]; then
    echo "usage: $0 <node-name> <description>"
    exit
fi

WORKING_DIR=`pwd`

###
### FIND PREFIX
###
PREFIX="NOT-FOUND"
while [[ `pwd` != "/" ]]; do    
    if [[ -f 'CMakeLists.txt' ]]; then
        PREFIX=`pwd`/
        break
    fi
    
    cd ..
done
if [[ $PREFIX == "NOT-FOUND" ]]; then
    echo "ERROR: cannot locate CMakeLists.txt"
    exit
fi

cd $PREFIX

DIR=${WORKING_DIR:${#PREFIX}}

###
### CMAKELISTS EXISTS, BEGIN PARSING
###
CMAKELIST=CMakeLists.txt

###
### TEST IF NODE NAME IS FREE IN CMAKE LIST
###
if [[ `cat $CMAKELIST | grep $NODE_NAME.cpp | wc -l` != 0 ]]; then
    echo "ERROR: $NODE_NAME.cpp already exists in CMakeLists.txt"
    exit
fi


###
### FIND PACKAGE XML
###
PACKAGEXML=package.xml
if [[ ! -f $PACKAGEXML ]]; then
    echo "ERROR: cannot locate the plugin xml file: $PACKAGEXML"
    exit
fi



###
### FIND PLUGIN XML INSIDE PACKAGE XML
###
PLUGINXML=$(cat $PACKAGEXML | grep "csapex" | grep "plugin=" | grep -Po 'plugin="\K[^"]*' | \
            sed -e "s|\${prefix}|${PREFIX}|")
if [[ ! -f $PLUGINXML ]]; then
    echo "ERROR: cannot locate the plugin xml file: $PLUGINXML"
    exit
fi

###
### TEST IF NODE NAME IS FREE IN PACKAGEXML
###
if [[ `cat $PLUGINXML | grep "type=.*$NODE_NAME" | wc -l` != 0 ]]; then
    echo "ERROR: $NODE_NAME already exists in $PLUGINXML"
    exit
fi


###
### FIND NAME OF THE PLUGIN LIBRARY
###
LIBRARY=$(cat $PLUGINXML | grep "<library" | grep -Po 'path="lib\K[^"]*')

if [[ ! $LIBRARY ]]; then
    echo "ERROR: cannot find library name in $PLUGINXML"
    exit
fi


###
### EVERYTHING THERE -> CREATE
###

NEW_FILE="$FILE_NAME.cpp"
NEW_FILE_H="$FILE_NAME.h"
NEW_XML_1="<class type=\"csapex::$NAMESPACE$NODE_NAME\" base_class_type=\"csapex::Node\">"
NEW_XML_2="  <description>$DESCRIPTION</description>"
NEW_XML_3="<\/class>"

###
### GENERATE HEADER
###

# convert camel case to upper case + underscore
GUARD=$(sed -e 's/\([A-Z]\)/_\L\1/g' -e 's/^_//' <<< $NODE_NAME)
GUARD=$(echo $GUARD | tr '[:lower:]' '[:upper:]')
GUARD="${GUARD}_H"

echo "#ifndef $GUARD
#define $GUARD

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {
$NAMESPACE_BEGIN

class $NODE_NAME : public csapex::Node
{
public:
    $NODE_NAME();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_;
    Output* out_;
};
$NAMESPACE_END

}

#endif // $GUARD" > $DIR/$NEW_FILE_H


###
### GENERATE SOURCE
###

echo "/// HEADER
#include \"$NEW_FILE_H\"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::${NAMESPACE}${NODE_NAME}, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
$NAMESPACE_USE

$NODE_NAME::$NODE_NAME()
{
}

void $NODE_NAME::setupParameters()
{
    // addParameter(param::ParameterFactory::declareRange(\"range\", 0.01, 1.0, 0.2, 0.01), update);

    //std::map<std::string, int> featuremix = boost::assign::map_list_of
    //        (\"standard\", 0)
    //        (\"no foo\", 1)
    //        (\"no bar\", 2);

    // addParameter(param::ParameterFactory::declareParameterSet<int>(\"feature mix\", featuremix));
}

void $NODE_NAME::setup()
{
    //in_  = modifier_->addInput<TYPE>(LABEL);
    //out_ = modifier_->addOutput<TYPE>(LABEL);
}

void $NODE_NAME::process()
{
}
"> $DIR/$NEW_FILE




###
### MODIFY CMAKELISTS
###
ENTRY="    $DIR/$NEW_FILE"
sed -i -e "/add_library.*$LIBRARY\s*/{
        $!{ N ; s|\(add_library.*$LIBRARY\s*\)\n\(.*\)|\\1\n$ENTRY\n\\2|
          }
         }" $CMAKELIST
                         
###
### MODIFY PLUGINXML
###
ENTRY="    $NEW_FILE"
sed -i -e "/<library.*$LIBRARY.*/{
         $!{ N ; s|\(<library.*$LIBRARY.*\)\n\(.*\)|\\1\n$NEW_XML_1\n$NEW_XML_2\n$NEW_XML_3\n\\2|
           }
         }" $PLUGINXML
                         

