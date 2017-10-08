HANDLE_ACCESSOR (GetDebugDescription,          std::string, getDebugDescription)
HANDLE_ACCESSOR (CanStartStepping,             bool,        canStartStepping)
HANDLE_ACCESSOR (IsActive,                     bool,        isActive)
HANDLE_ACCESSOR (IsSource,                     bool,        isSource)
HANDLE_ACCESSOR (IsSink,                       bool,        isSink)
HANDLE_ACCESSOR (IsProcessingNothingMessages,  bool,        isProcessingNothingMessages)
HANDLE_ACCESSOR (GetErrorLevel,                ErrorState::ErrorLevel, errorLevel)
HANDLE_ACCESSOR (GetErrorMessage,              std::string, errorMessage)
HANDLE_ACCESSOR (GetExecutionState,            ExecutionState, getExecutionState)

// TODO: make these dynamic
HANDLE_ACCESSOR (GetLabel,                     std::string, getLabel)
HANDLE_ACCESSOR (GetExecutionFrequency,        double, getExecutionFrequency)
HANDLE_ACCESSOR (GetMaximumFrequency,          double, getMaximumFrequency)
HANDLE_ACCESSOR (GetNodeCharacteristics,       NodeCharacteristics, getNodeCharacteristics)

HANDLE_ACCESSOR (IsProcessingEnabled,          bool, isProcessingEnabled)
HANDLE_ACCESSOR (HasVariadicInputs,            bool, hasVariadicInputs)
HANDLE_ACCESSOR (HasVariadicOutputs,           bool, hasVariadicOutputs)
HANDLE_ACCESSOR (HasVariadicEvents,            bool, hasVariadicEvents)
HANDLE_ACCESSOR (HasVariadicSlots,             bool, hasVariadicSlots)
HANDLE_ACCESSOR (GetInputs,                    std::vector<ConnectorDescription>, getInputs)
HANDLE_ACCESSOR (GetOutputs,                   std::vector<ConnectorDescription>, getOutputs)
HANDLE_ACCESSOR (GetEvents,                    std::vector<ConnectorDescription>, getEvents)
HANDLE_ACCESSOR (GetSlots,                     std::vector<ConnectorDescription>, getSlots)
HANDLE_ACCESSOR (GetInternalInputs,            std::vector<ConnectorDescription>, getInternalInputs)
HANDLE_ACCESSOR (GetInternalOutputs,           std::vector<ConnectorDescription>, getInternalOutputs)
HANDLE_ACCESSOR (GetInternalEvents,            std::vector<ConnectorDescription>, getInternalEvents)
HANDLE_ACCESSOR (GetInternalSlots,             std::vector<ConnectorDescription>, getInternalSlots)
HANDLE_ACCESSOR (GetExternalInputs,            std::vector<ConnectorDescription>, getExternalInputs)
HANDLE_ACCESSOR (GetExternalOutputs,           std::vector<ConnectorDescription>, getExternalOutputs)
HANDLE_ACCESSOR (GetExternalEvents,            std::vector<ConnectorDescription>, getExternalEvents)
HANDLE_ACCESSOR (GetExternalSlots,             std::vector<ConnectorDescription>, getExternalSlots)
HANDLE_ACCESSOR (IsProfiling,                  bool, isProfiling)
HANDLE_ACCESSOR (IsError,                      bool, isError)


HANDLE_STATIC_ACCESSOR (IsVariadic,            bool,        isVariadic)
HANDLE_STATIC_ACCESSOR (IsGraph,               bool,        isGraph)
HANDLE_STATIC_ACCESSOR (GetSubgraphAUUID,      AUUID,       getSubgraphAUUID)
HANDLE_STATIC_ACCESSOR (GetType,               std::string, getType)

HANDLE_SIGNAL (MessagesProcessed,              messages_processed)
HANDLE_SIGNAL (NodeStateChanged,               node_state_changed)
HANDLE_SIGNAL (ParametersChanged,              parameters_changed)
HANDLE_SIGNAL (Destroyed,                      destroyed)


#undef HANDLE_DYNAMIC_ACCESSOR
#undef HANDLE_STATIC_ACCESSOR
#undef HANDLE_ACCESSOR
#undef HANDLE_SIGNAL
