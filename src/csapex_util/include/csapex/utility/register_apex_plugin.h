#ifndef REGISTER_APEX_PLUGIN_H
#define REGISTER_APEX_PLUGIN_H

/// SYSTEM
#include <csapex/utility/suppress_warnings_start.h>
#include <class_loader/class_loader.h>
#include <csapex/utility/suppress_warnings_end.h>

#define CSAPEX_REGISTER_CLASS(name, parent) \
    CLASS_LOADER_REGISTER_CLASS(name,parent)
	
#endif // REGISTER_PLUGIN_H
