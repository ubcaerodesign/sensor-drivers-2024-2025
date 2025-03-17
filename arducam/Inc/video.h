/*
 * video.h
 *
 *  Created on: Mar 16, 2025
 *      Author: ryanz
 */

#ifndef INC_VIDEO_H_
#define INC_VIDEO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "ArducamCamera.h"

/* Exported define -----------------------------------------------------------*/
#define FRAMES_NUM        200     // Maximum number of frames to capture
#define AVIOFFSET         240     // AVI header size
#define FRAME_RATE        10      // Frames per second

/* Exported variables --------------------------------------------------------*/
extern const uint8_t avi_header[AVIOFFSET];
extern const uint8_t zero_buf[4];

/* Public function prototypes ------------------------------------------------*/

/**
  * @brief  Writes a 32-bit integer to file in little-endian format
  * @param  i: 32-bit value to write
  * @param  fd: Pointer to file object
  * @retval None
  */
void print_quartet(uint32_t i, FIL* fd);

/**
  * @brief  Generate a unique filename for a video
  * @param  filename: Buffer to store the generated filename
  * @param  size: Size of the filename buffer
  * @retval None
  */
void generateVideoFilename(char* filename, size_t size);

/**
  * @brief  Record video from the camera and save it as an AVI file
  * @param  camera: Pointer to the ArducamCamera object
  * @param  numFrames: Number of frames to capture (limited by FRAMES_NUM)
  * @param  resolution: Camera resolution mode
  * @retval FRESULT: FR_OK if successful, error code otherwise
  */
FRESULT cameraRecordVideo(ArducamCamera* camera, uint16_t numFrames, CAM_IMAGE_MODE resolution);

#ifdef __cplusplus
}
#endif

#endif /* INC_VIDEO_H_ */
