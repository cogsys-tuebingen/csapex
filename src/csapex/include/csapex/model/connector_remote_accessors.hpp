#ifdef HANDLE_ACCESSOR

HANDLE_ACCESSOR(int, getCount, GetCount)
HANDLE_ACCESSOR(bool, canOutput, CanOutput)
HANDLE_ACCESSOR(bool, canInput, CanInput)
HANDLE_ACCESSOR(bool, isOutput, IsOutput)
HANDLE_ACCESSOR(bool, isInput, IsInput)
HANDLE_ACCESSOR(bool, isOptional, IsOptional)
HANDLE_ACCESSOR(bool, isSynchronous, IsSynchronous)
HANDLE_ACCESSOR(bool, isVirtual, IsVirtual)
HANDLE_ACCESSOR(bool, isParameter, IsParameter)
HANDLE_ACCESSOR(bool, isGraphPort, IsGraphPort)
HANDLE_ACCESSOR(bool, isEssential, IsEssential)
HANDLE_ACCESSOR(std::string, getLabel, GetLabel)
HANDLE_ACCESSOR(ConnectorType, getConnectorType, GetConnectorType)
HANDLE_ACCESSOR(ConnectorDescription, getDescription, GetDescription)
HANDLE_ACCESSOR(bool, isEnabled, IsEnabled)
HANDLE_ACCESSOR(int, sequenceNumber, GetSequenceNumber)
HANDLE_ACCESSOR(int, countConnections, GetConnectionCount)
HANDLE_ACCESSOR(bool, hasActiveConnection, HasActiveConnection)
HANDLE_ACCESSOR(bool, isConnected, IsConnected)
HANDLE_ACCESSOR(std::string, makeStatusString, MakeStatusString)
HANDLE_ACCESSOR(TokenDataConstPtr, getType, GetType)

#else
#pragma message "Cannot generate accessors!"
#endif
