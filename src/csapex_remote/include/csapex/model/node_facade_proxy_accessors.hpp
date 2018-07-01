// TODO: make these dynamic
HANDLE_ACCESSOR(GetDebugDescription, std::string, getDebugDescription)
HANDLE_ACCESSOR(CanStartStepping, bool, canStartStepping)
HANDLE_ACCESSOR(IsActive, bool, isActive)
HANDLE_ACCESSOR(IsSource, bool, isSource)
HANDLE_ACCESSOR(IsSink, bool, isSink)
HANDLE_ACCESSOR(IsProcessingNothingMessages, bool, isProcessingNothingMessages)
HANDLE_ACCESSOR(GetExecutionState, ExecutionState, getExecutionState)

HANDLE_DYNAMIC_ACCESSOR(GetLabel, label_changed, std::string, getLabel)
HANDLE_DYNAMIC_ACCESSOR(GetSchedulerId, scheduler_changed, int, getSchedulerId)

HANDLE_ACCESSOR(GetExecutionFrequency, double, getExecutionFrequency)
HANDLE_ACCESSOR(GetMaximumFrequency, double, getMaximumFrequency)
HANDLE_ACCESSOR(GetNodeCharacteristics, NodeCharacteristics, getNodeCharacteristics)
HANDLE_ACCESSOR(IsProcessingEnabled, bool, isProcessingEnabled)
HANDLE_DYNAMIC_ACCESSOR(GetExternalInputs, external_inputs_changed, std::vector<ConnectorDescription>, getExternalInputs)
HANDLE_DYNAMIC_ACCESSOR(GetExternalOutputs, external_outputs_changed, std::vector<ConnectorDescription>, getExternalOutputs)
HANDLE_DYNAMIC_ACCESSOR(GetExternalEvents, external_events_changed, std::vector<ConnectorDescription>, getExternalEvents)
HANDLE_DYNAMIC_ACCESSOR(GetExternalSlots, external_slots_changed, std::vector<ConnectorDescription>, getExternalSlots)
HANDLE_DYNAMIC_ACCESSOR(GetInternalInputs, internal_inputs_changed, std::vector<ConnectorDescription>, getInternalInputs)
HANDLE_DYNAMIC_ACCESSOR(GetInternalOutputs, internal_outputs_changed, std::vector<ConnectorDescription>, getInternalOutputs)
HANDLE_DYNAMIC_ACCESSOR(GetInternalEvents, internal_events_changed, std::vector<ConnectorDescription>, getInternalEvents)
HANDLE_DYNAMIC_ACCESSOR(GetInternalSlots, internal_slots_changed, std::vector<ConnectorDescription>, getInternalSlots)
HANDLE_ACCESSOR(IsProfiling, bool, isProfiling)

HANDLE_STATIC_ACCESSOR(HasVariadicInputs, bool, hasVariadicInputs)
HANDLE_STATIC_ACCESSOR(HasVariadicOutputs, bool, hasVariadicOutputs)
HANDLE_STATIC_ACCESSOR(HasVariadicEvents, bool, hasVariadicEvents)
HANDLE_STATIC_ACCESSOR(HasVariadicSlots, bool, hasVariadicSlots)
HANDLE_STATIC_ACCESSOR(IsVariadic, bool, isVariadic)
HANDLE_STATIC_ACCESSOR(IsGraph, bool, isGraph)
HANDLE_STATIC_ACCESSOR(GetSubgraphAUUID, AUUID, getSubgraphAUUID)
HANDLE_STATIC_ACCESSOR(GetType, std::string, getType)

HANDLE_SIGNAL(MessagesProcessed, messages_processed)
HANDLE_SIGNAL(ParametersChanged, parameters_changed)
HANDLE_SIGNAL(ParameterSetChanged, parameter_set_changed)
HANDLE_SIGNAL(ActivationChanged, activation_changed)
HANDLE_SIGNAL(Destroyed, destroyed)

#undef HANDLE_DYNAMIC_ACCESSOR
#undef HANDLE_STATIC_ACCESSOR
#undef HANDLE_ACCESSOR
#undef HANDLE_SIGNAL
