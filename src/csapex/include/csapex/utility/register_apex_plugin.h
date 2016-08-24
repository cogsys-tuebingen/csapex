#ifndef REGISTER_APEX_PLUGIN_H
#define REGISTER_APEX_PLUGIN_H

/// SYSTEM
#include <class_loader/class_loader_register_macro.h>

#define CSAPEX_REGISTER_CLASS(name, parent) \
    CLASS_LOADER_REGISTER_CLASS(name,parent)
	
#endif // REGISTER_PLUGIN_H
