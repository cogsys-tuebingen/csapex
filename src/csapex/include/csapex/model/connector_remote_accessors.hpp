HANDLE_STATIC_ACCESSOR (IsOutput,            bool, isOutput)
HANDLE_STATIC_ACCESSOR (IsInput,             bool, isInput)
HANDLE_STATIC_ACCESSOR (IsOptional,          bool, isOptional)
HANDLE_STATIC_ACCESSOR (IsSynchronous,       bool, isSynchronous)
HANDLE_STATIC_ACCESSOR (IsVirtual,           bool, isVirtual)
HANDLE_STATIC_ACCESSOR (IsParameter,         bool, isParameter)
HANDLE_STATIC_ACCESSOR (IsGraphPort,         bool, isGraphPort)

// TODO: make these dynamic
HANDLE_STATIC_ACCESSOR (GetConnectorType,    ConnectorType, getConnectorType)
HANDLE_STATIC_ACCESSOR (MaxConnectionCount,  int, maxConnectionCount)
HANDLE_STATIC_ACCESSOR (GetDescription,      ConnectorDescription, getDescription)
HANDLE_STATIC_ACCESSOR (IsConnected,         bool, isConnected)

HANDLE_DYNAMIC_ACCESSOR (IsEssential,    essential_changed, bool, isEssential)
HANDLE_DYNAMIC_ACCESSOR (GetLabel,       labelChanged, std::string, getLabel)
HANDLE_DYNAMIC_ACCESSOR (IsEnabled,      enabled_changed, bool, isEnabled)
HANDLE_DYNAMIC_ACCESSOR (GetType,        typeChanged, TokenDataConstPtr, getType)

HANDLE_ACCESSOR (GetCount,            int, getCount)
HANDLE_ACCESSOR (GetSequenceNumber,   int, sequenceNumber)
HANDLE_ACCESSOR (GetConnectionCount,  int, countConnections)
HANDLE_ACCESSOR (HasActiveConnection, bool, hasActiveConnection)
HANDLE_ACCESSOR (MakeStatusString,    std::string, makeStatusString)
HANDLE_ACCESSOR (GetConnectedPorts,   std::vector<UUID>, getConnectedPorts)

#undef HANDLE_DYNAMIC_ACCESSOR
#undef HANDLE_STATIC_ACCESSOR
#undef HANDLE_ACCESSOR
