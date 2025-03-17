/*
 * image.h
 *
 *  Created on: Mar 16, 2025
 *      Author: ryanz
 */

#ifndef INC_IMAGE_H_
#define INC_IMAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "ArducamCamera.h"

/* Public function prototypes ------------------------------------------------*/

/**
  * @brief  Setup camera with initial configurations
  * @param  camera: Pointer to the ArducamCamera object
  * @retval None
  */
void setupCamera(ArducamCamera* camera);

/**
  * @brief  Generate a unique filename for an image
  * @param  filename: Buffer to store the generated filename
  * @param  size: Size of the filename buffer
  * @retval None
  */
void generateImgName(char* filename, size_t size);

/**
  * @brief  Capture an image from the camera and save it to the SD card
  * @param  camera: Pointer to the ArducamCamera object
  * @retval None
  */
void cameraCaptureAndSaveImage(ArducamCamera* camera);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMAGE_H_ */
