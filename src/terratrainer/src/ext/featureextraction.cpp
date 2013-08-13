#include "imagefeatureextraction.h"
#include <iostream>
using namespace std;

#define FEATURE_EXTRACTION_VERSION "1.1"

void printUsage()
{
    cout << "feature_extraction " << FEATURE_EXTRACTION_VERSION << endl;
    cout << "Usage: feature_extraction [directory with original images] [directory with ground-truth images] [LTP/TSURF/TSURF+] [size] [parameter] [USE_POS]" << endl;
}

int main(int argc, char *argv[])
{
    if (argc < 6)
    {
        printUsage();
        return 1;
    }

    QString colorsFile = QString(argv[2]) + "/colors.txt";
    string desc = argv[3];
    int size = atoi(argv[4]);
    float parameter = atof(argv[5]);
    bool usePosition = (argc > 6 && string(argv[6]) == "USE_POS");

    ImageFeatureExtraction::Descriptor descriptor;

    if (desc == "LTP")
    {
        descriptor = ImageFeatureExtraction::DESCRIPTOR_LTP;
    }
    else if (desc == "TSURF")
    {
        descriptor = ImageFeatureExtraction::DESCRIPTOR_TSURF;
    }
    else if (desc == "XSURF")
    {
        descriptor = ImageFeatureExtraction::DESCRIPTOR_XSURF;
    }
    else
    {
        cerr << "Unknown descriptor " << desc << endl;
        return 1;
    }

    ImageFeatureExtraction ife;
    return ife.extract(colorsFile, argv[1], argv[2], descriptor, size, parameter, usePosition);
}
