HANDLE_DYNAMIC_ACCESSOR (IsPaused,          paused, bool, isPaused)

HANDLE_ACCESSOR (MakeStatusString,          std::string, makeStatusString)

HANDLE_SIGNAL (Stopped,                     stopped)

#undef HANDLE_DYNAMIC_ACCESSOR
#undef HANDLE_STATIC_ACCESSOR
#undef HANDLE_ACCESSOR
#undef HANDLE_SIGNAL
