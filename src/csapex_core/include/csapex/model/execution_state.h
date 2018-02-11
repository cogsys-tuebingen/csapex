#ifndef EXECUTION_STATE_H
#define EXECUTION_STATE_H

namespace csapex
{

enum class ExecutionState {
    IDLE,
    ENABLED,
    FIRED,
    PROCESSING,
    UNKNOWN
};

}

#endif // EXECUTION_STATE_H
