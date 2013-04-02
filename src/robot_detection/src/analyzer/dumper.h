#ifndef DUMPER_H
#define DUMPER_H

/// COMPONENT
#include "trainer.h"

/**
 * @brief The Dumper class saves images to the hard disk
 */
class Dumper : public Trainer
{
public:
    /**
     * @brief Dumper
     * @throws if something went wrong
     */
    Dumper();

public:
    /**
     * @brief stopTraining Override: Set training-stopped state
     */
    void stopTraining();

protected:
    /**
     * @brief training Is called in the training state
     * @param frame the current frame
     */
    void training(Frame::Ptr frame);

    /**
     * @brief dumpCurrentRoiAsImage Saves the current region of interest as an image
     */
    void dumpCurrentRoiAsImage();

    // @TODO: encapsulate
public:
    bool dump_vj_data;
    bool dump_feature_data;
    bool dump_reference_images;

protected:
    long feature;
};

#endif // DUMPER_H
