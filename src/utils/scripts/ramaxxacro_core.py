#! /usr/bin/env python
# @brief wraps xacro to add parameter replacement
#        #{name} is replaced by _value_, if argument name:="_value_" is given
# @author buck

import sys
import subprocess

def main():
    if len(sys.argv) < 2:
        print "usage", sys.argv[0], " <ramaxxacro file> [optional <substitution pairs>]"
        print "  substitution pair:    name_without_spaces:=\"the value\""
        exit()
    
    substitutions = dict()
    
    if len(sys.argv) > 2:
        for pair in sys.argv[2:]:
            [key, val] = pair.split(":=")
            substitutions[key] = val

    command = "rosrun xacro xacro.py " + sys.argv[1]
    text = subprocess.check_output(command, shell=True)
    
    for key in substitutions:
        text = text.replace("#{" + key + "}", substitutions[key])
    
    print text
