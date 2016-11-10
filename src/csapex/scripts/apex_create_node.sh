#!/bin/bash -l

FULLNAME=$1

if [[ $(grep "::" <<< $1) ]]; then
    arrIN=(${1//::/ })
    NODE_NAME=${arrIN[-1]}
    NAMESPACES=(${arrIN[@]})
    unset NAMESPACES[${#NAMESPACES[@]}-1]
else
    NODE_NAME=$1
    NAMESPACES=("csapex")
    FULLNAME="csapex::$FULLNAME"
fi


OPENING_NS=$""
CLOSING_NS=$""

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
CMAKELIST=${PREFIX}CMakeLists.txt

###
### TEST IF NODE NAME IS FREE IN CMAKE LIST
###
if [[ `cat $CMAKELIST | grep "/[[:space:]]*$FILE_NAME" | wc -l` != 0 ]]; then
    echo "ERROR: $FILE_NAME already exists in CMakeLists.txt"
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
if [[ `cat $PLUGINXML | grep "type=.*$FULLNAME\"" | wc -l` != 0 ]]; then
    echo "ERROR: $NODE_NAME already exists in $PLUGINXML"
    exit
fi

###
### DETERMINE THE PROJECT NAME
###
PROJECT_NAME=$(cat $CMAKELIST | grep "project" | sed "s/project(\(.*\))/\\1/");

###
### FIND NAME OF THE PLUGIN LIBRARY
###
LIBRARY=($(cat $PLUGINXML | grep "<library" | grep -Po 'path="lib\K[^"]*'))

if [[ ! $LIBRARY ]]; then
    echo "ERROR: cannot find library name in $PLUGINXML"
    exit
fi

if [[ ${#LIBRARY[@]} > 1 ]]; then
    echo "There are multiple libraries, please select the target:"
    select LIB in ${LIBRARY[@]};
    do
        LIBRARY=$LIB
        echo "You picked $LIBRARY"
        break
    done
else
    echo "Target library is $LIBRARY"
fi

###
### EVERYTHING THERE -> CREATE
###

NEW_FILE="$FILE_NAME.cpp"
NEW_XML_1="<class type=\"$FULLNAME\" base_class_type=\"csapex::Node\">"
NEW_XML_2="  <description>$DESCRIPTION</description>"
NEW_XML_3="<\/class>"



###
### GENERATE SOURCE
###


for ns in "${NAMESPACES[@]}"
do
    OPENING_NS="${OPENING_NS}namespace $ns
{
"
    CLOSING_NS="} // $ns
$CLOSING_NS"
done

echo "
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

using namespace csapex;
using namespace csapex::connection_types;

$OPENING_NS

class $NODE_NAME : public Node
{
public:
    $NODE_NAME()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<std::string>(\"Input\");
        out_ = modifier.addOutput<std::string>(\"Output\");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        std::string value = msg::getValue<std::string>(in_);

        MessageConstPtr message = msg::getMessage<GenericValueMessage<std::string>>(in_);
        apex_assert(message);
        msg::publish(out_, value + \"!\");
    }

private:
    Input* in_;
    Output* out_;

};

$CLOSING_NS

CSAPEX_REGISTER_CLASS($FULLNAME, csapex::Node)
"> $DIR/$NEW_FILE




###
### MODIFY CMAKELISTS
###
WS=$(grep "add_library.*$LIBRARY" $CMAKELIST -A 1 | tail -n 1 | cut -d's' -f1 | sed 's/ /\\ /')
ENTRY="${WS}$DIR/$NEW_FILE"
sed -i "/add_library.*$LIBRARY\s*$/a $ENTRY"  $CMAKELIST
sed -i "/add_library.*$LIBRARY\s*SHARED\s*$/a $ENTRY"  $CMAKELIST

LIBRARY_VAR=$(echo $LIBRARY | sed "s/${PROJECT_NAME}/\${PROJECT_NAME}/")

WS=$(grep "add_library.*$LIBRARY_VAR" $CMAKELIST -A 1 | tail -n 1 | cut -d's' -f1 | sed 's/ /\\ /')
ENTRY="${WS}$DIR/$NEW_FILE"
sed -i "/add_library.*$LIBRARY_VAR\s*$/a $ENTRY"  $CMAKELIST
sed -i "/add_library.*$LIBRARY_VAR\s*SHARED\s*$/a $ENTRY"  $CMAKELIST
                         
###
### MODIFY PLUGINXML
###
ENTRY="$NEW_XML_1\n$NEW_XML_2\n$NEW_XML_3"
sed -i -e "/<library.*$LIBRARY/a $ENTRY"  $PLUGINXML
