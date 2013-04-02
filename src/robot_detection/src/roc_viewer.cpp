/// PROJECT
#include <common/global.hpp>
#include <roc/roc_creator.h>

/// SYSTEM
#include <iostream>

int main(int argc, char** argv)
{
    if(argc != 2) {
        ERROR("usage: " << argv[0] << " <path-to-roc-file>");
        ERROR_ << "you provided:";
        for(int i = 1; i < argc; ++i) {
            ERROR_ << " " << argv[i];
        }
        ERROR_.endl();
        return 1;
    }

    RocCreator creator;
    if(creator.load(argv[1])) {
        creator.displayInteractive();
    }
}
