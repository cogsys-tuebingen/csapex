#include "randomforest.h"

int main(int argc, char *argv[])
{
    if(argc < 2) {
        std::cerr << "Give a valid forest path!" << std::endl;
        return 1;
    }
    RandomForest f;
    f.load(argv[1]);
    return 0;
}
