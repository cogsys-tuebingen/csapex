/// HEADER
#include "trainer_adapter.h"

/// PROJECT
#include <analyzer/trainer.h>

TrainerAdapter::TrainerAdapter(Trainer& trainer)
    : AnalyzerAdapter(trainer), trainer(trainer)
{
}

TrainerAdapter::~TrainerAdapter()
{
}
