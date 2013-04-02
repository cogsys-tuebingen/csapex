/*

  Authors:
  --------
  Olivier Barnich and
  Marc Van Droogenbroeck <m.vandroogenbroeck@ulg.ac.be>

  Licence:
  --------
  ViBe is covered by a patent (see http://www.ulg.ac.be/telecom/research/vibe).
  We do not provide the source code but provide an object file and an interface.

  Permission to use ViBe without payment of fee is granted
  for nonprofit educational and research purposes only.

  This work may not be copied or reproduced in whole or in part for any
  purpose.

  Copying, reproduction, or republishing for any purpose shall require
  a license. Please contact the author in such cases.
  All the code is provided without any guarantee.


 How to refer to us:
 -------------------

 If you want to refer to this work, please cite the following publications:

 O. Barnich and M. Van Droogenbroeck. ViBe: a powerful random technique
 to estimate the background in video sequences. In International
 Conference on Acoustics, Speech, and Signal Processing (ICASSP 2009),
 pages 945-948, April 2009.

 http://www.ulg.ac.be/telecom/publi/publications/mvd/Barnich2009ViBe.pdf²

 @inproceedings{Barnich2009ViBe,
   title = {{ViBe}: a powerful random technique to estimate the background in video sequences},
   author = {O. Barnich and M. {Van Droogenbroeck}},
   booktitle = {International Conference on Acoustics, Speech, and Signal Processing (ICASSP 2009)},
   year = {2009},
   pages = {945-948},
   month = {April},
   pdf = {http://orbi.ulg.ac.be/bitstream/2268/12087/1/Barnich2009ViBe.pdf},
   url = {http://hdl.handle.net/2268/12087}
 }

  How to use this code :
  ----------------------
  - INIT
     - First call libvibeModelNew to generate a new instance of a ViBe Model:
       ie. model = libvibeModelNew();

     - Then use the provided "set" function to tune the parameters
       nb: This step is optional!
       ie. libvibeModelSetNumberOfSamples(model, 20);

     - Finally, call libvibeModelAllocInit_8u_C1R (for monochromatic images)
       or libvibeModelAllocInit_8u_C3R for trichromatic (ie. rgbrgbrgbrgb...)
       images to allocate and initialize the model with the help of
       *image_data that points to the pixel values of the first image of your
       video stream.
       ie. libvibeModelAllocInit_8u_C1R(model, image, width, height, stride);

   - FOR EACH NEW FRAME
     You must call libvibeModelUpdate_8u_C1R or libvibeModelUpdate_8u_C3R
     that performs the classification of the pixels of the new image and
     updates the model according to them.
     ie. libvibeModelUpdate_8u_C1R(model, current_image, segmentation_map);
     nb. *segmentation_map points to the pixel values of a monochromatic image
         where the results of the classification will be stored. Its width,
	 height and stride must match those of your input images. The same
	 goes for the dimensions of *image_data.

  - CLEANUP
    A call to libvibeModelFree() frees all the allocated memory.


  Example of a code (in c99) for your main file:
  ----------------------------------------------
  #include "vibe-background.h"

  int main(int argc, char **argv){
     // Get a model data structure
     vibeModel_t *model = libvibeModelNew();

     // Acquire your first (grayscale) image
     // nb: stride is te number of bytes from the start of one row of the image
     //     to the start of the next row.
     uint8_t *image_data = acquire_image(stream);
     int32_t width = get_image_width(stream);
     int32_t height = get_image_height(stream);
     int32_t stride = get_image_stride(stream);

     // Allocate another image to store the resulting segmentation map
     uint8_t *segmentation_map = malloc(stride * height);

     // Allocate the model and initialize it with the first image
     libvibeModelAllocInit_8u_C1R(model, image_data, width, height, stride);

     // Process all the following frames of your stream
     // nb. the results will be stored in *segmentation_map
     while(!finished(stream)){
        image_data = acquire_image(stream);
        libvibeModelUpdate_8u_C1R(model, image_data, segmentation_map);
     }

     // Cleanup allocated memory
     libvibeModelFree(model);
  }

  How to compile your file and link it to vibe-background.o
  ---------------------------------------------------------
  gcc -o vibe-demo yourMainFile.c vibe-background.o
  nb. vibe-background.o was compiled using the following command:
  gcc -c -ansi -Wall -Werror -pedantic vibe-background.c

*/

#ifndef _VIBE_BACKGROUND_H_
#define _VIBE_BACKGROUND_H_

/* For compilation with g++ */
#ifdef __cplusplus
extern "C"
{
#endif
    /* end of addition for compilation with g++ */

#include <stdlib.h>
#include <stdio.h>

#define COLOR_BACKGROUND   0
#define COLOR_FOREGROUND 255

    typedef unsigned char uint8_t;
    typedef int int32_t;
    typedef unsigned int uint32_t;


    typedef struct vibeModel vibeModel_t;

    /* Allocation of a new data structure where the background model
       will be stored */
    vibeModel_t* libvibeModelNew();

    /* ViBe uses several parameters
       You can print and change them if you want. However, default
       value should meet your needs if about all the videos.  */
    uint32_t libvibeModelPrintParameters(const vibeModel_t* model);

    int32_t libvibeModelSetNumberOfSamples(vibeModel_t* model,
                                           const uint32_t numberOfSamples);
    uint32_t libvibeModelGetNumberOfSamples(const vibeModel_t* model);

    int32_t libvibeModelSetMatchingThreshold(vibeModel_t* model,
            const uint32_t matchingThreshold);
    uint32_t libvibeModelGetMatchingThreshold(const vibeModel_t* model);

    int32_t libvibeModelSetMatchingNumber(vibeModel_t* model,
                                          const uint32_t matchingNumber);
    uint32_t libvibeModelGetMatchingNumber(const vibeModel_t* model);

    int32_t libvibeModelSetUpdateFactor(vibeModel_t* model,
                                        const uint32_t updateFactor);
    uint32_t libvibeModelGetUpdateFactor(const vibeModel_t* model);

    /* The 2 following functions allocate the required memory according to the
       model parameters and the dimensions of the input images.
       You must use the "C1R" function for grayscale images and the "C3R" for color
       images.
       The pixel values of color images  are arranged in the following order
       RGBRGBRGB... (or HSVHSVHSVHSVHSVHSV...)
       These 2 functions also initialize the background model using the content
       of *image_data which is the pixel buffer of the first image of your stream.

       nb: stride is te number of bytes from the start of one row of the image
       to the start of the next row. */

    int32_t libvibeModelAllocInit_8u_C1R(vibeModel_t* model,
                                         const uint8_t* image_data,
                                         const uint32_t width,
                                         const uint32_t height,
                                         const uint32_t stride);
    int32_t libvibeModelAllocInit_8u_C3R(vibeModel_t* model,
                                         const uint8_t* image_data,
                                         const uint32_t width,
                                         const uint32_t height,
                                         const uint32_t stride);

    /* These 2 functions perform 2 operations:
         - they classify the pixels *image_data using the provided model and store
           the results in *segmentation_map
         - they update *model according to these results and the content of
           *image_data
       You must use the "C1R" function for grayscale images and the "C3R" for color
       images */
    int32_t libvibeModelUpdate_8u_C1R(vibeModel_t* model,
                                      const uint8_t* image_data,
                                      uint8_t* segmentation_map);
    int32_t libvibeModelUpdate_8u_C3R(vibeModel_t* model,
                                      const uint8_t* image_data,
                                      uint8_t* segmentation_map);

    /* This function frees all the memory allocated by libvibeModelNew and
       libvibeModelAllocInit_8u_CxR */
    int32_t libvibeModelFree(vibeModel_t* model);

    /* For compilation with g++ */
#ifdef __cplusplus
}
#endif
/* end of addition for compilation with g++ */

#endif
