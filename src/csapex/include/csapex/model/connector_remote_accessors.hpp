#ifdef HANDLE_ACCESSOR

HANDLE_ACCESSOR(GetCount,            int, getCount)
HANDLE_ACCESSOR(IsOutput,            bool, isOutput)
HANDLE_ACCESSOR(IsInput,             bool, isInput)
HANDLE_ACCESSOR(IsOptional,          bool, isOptional)
HANDLE_ACCESSOR(IsSynchronous,       bool, isSynchronous)
HANDLE_ACCESSOR(IsVirtual,           bool, isVirtual)
HANDLE_ACCESSOR(IsParameter,         bool, isParameter)
HANDLE_ACCESSOR(IsGraphPort,         bool, isGraphPort)
HANDLE_ACCESSOR(IsEssential,         bool, isEssential)
HANDLE_ACCESSOR(GetLabel,            std::string, getLabel)
HANDLE_ACCESSOR(GetConnectorType,    ConnectorType, getConnectorType)
HANDLE_ACCESSOR(GetDescription,      ConnectorDescription, getDescription)
HANDLE_ACCESSOR(IsEnabled,           bool, isEnabled)
HANDLE_ACCESSOR(GetSequenceNumber,   int, sequenceNumber)
HANDLE_ACCESSOR(GetConnectionCount,  int, countConnections)
HANDLE_ACCESSOR(MaxConnectionCount,   int, maxConnectionCount)
HANDLE_ACCESSOR(HasActiveConnection, bool, hasActiveConnection)
HANDLE_ACCESSOR(IsConnected,         bool, isConnected)
HANDLE_ACCESSOR(MakeStatusString,    std::string, makeStatusString)
HANDLE_ACCESSOR(GetType,             TokenDataConstPtr, getType)
HANDLE_ACCESSOR(GetConnectedPorts,   std::vector<UUID>, getConnectedPorts)

#else
#pragma message "Cannot generate accessors!"
#endif
