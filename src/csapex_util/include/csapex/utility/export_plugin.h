#if WIN32
#define CSAPEX_EXPORT_PLUGIN __declspec(dllexport)
#else
#define CSAPEX_EXPORT_PLUGIN
#endif
