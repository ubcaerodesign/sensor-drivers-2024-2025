#ifndef SRC_ARDUCAM_H_
#define SRC_ARDUCAM_H_

#include "ArducamCamera.h"


/**
 * @brief Structure to encapsulate. Arducam Mega functionality
 */
typedef struct {
    ArducamCamera cameraInfo;
} Arducam;

//**********************************************
//!
//! @brief Constructor of camera class
//!
//! @param cs enable pin
//!
//**********************************************
ArducamCamera Arducam_Init(int cs);

//**********************************************
//!
//! @brief reset camera
//!
//**********************************************
CamStatus Arducam_Reset(Arducam* cam);
//**********************************************
//!
//! @brief Initialize the configuration of the camera module
//! @return Return operation status
//**********************************************

CamStatus Arducam_Begin(Arducam* cam);

//**********************************************
//!
//! @brief Start a snapshot with specified resolution and pixel format
//!
//! @param mode Resolution of the camera module
//! @param pixel_format Output image pixel format,which supports JPEG, RGB,
//! YUV
//!
//! @return Return operation status
//!
//! @note The mode parameter must be the resolution which the current camera
//! supported
//**********************************************
CamStatus Arducam_TakePicture(Arducam* cam, CAM_IMAGE_MODE mode, CAM_IMAGE_PIX_FMT pixel_format);

//**********************************************
//!
//! @brief Start multi capture with specified number of image.
//!
//! @param mode Resolution of the camera module
//! @param pixel_format Output image pixel format,which supports JPEG, RGB,
//! YUV
//! @param number Number of pictures taken
//!
//! @return Return operation status
//!
//!
//! @note The mode parameter must be the resolution which the current camera
//! supported
//**********************************************
CamStatus Arducam_TakeMultiPictures(Arducam* cam, CAM_IMAGE_MODE mode, CAM_IMAGE_PIX_FMT pixel_format, uint8_t number);

//**********************************************
//!
//! @brief Start preview with specified resolution mode.
//!
//! @param mode Resolution of the camera module
//!
//! @return Return operation status
//!
//! @note Before calling this function, you need to register the callback
//! function.The default image pixel format is JPEG
//**********************************************
CamStatus Arducam_StartPreview(Arducam* cam, CAM_VIDEO_MODE mode);

//**********************************************
//! @brief The camera's stream processing thread
//**********************************************
void Arducam_CaptureThread(Arducam* cam);

//**********************************************
//!
//! @brief Stop preview
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_StopPreview(Arducam* cam);
//**********************************************
//!
//! @brief Set the exposure mode
//!
//! @param   val `0` Turn on automatic exposure `1` Turn off automatic
//! exposure
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetAutoExposure(Arducam* cam, uint8_t val);
//**********************************************
//!
//! @brief Set the exposure time Manually
//!
//! @param   val Value of exposure line
//!
//! @return Return operation status
//!
//! @note Before calling this function, you need to turn off the auto
//! exposure function
//**********************************************
CamStatus Arducam_SetAbsoluteExposure(Arducam* cam, uint32_t val);
//**********************************************
//!
//! @brief Set the gain mode
//!
//! @param   val `0` turn on automatic gain `1` turn off automatic gain
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetAutoISOSensitive(Arducam* cam, uint8_t val);
//**********************************************
//!
//! @brief Set the gain time Manually
//!
//! @param  val Value of gain
//!
//! @return Return operation status
//!
//! @note Before calling this function, you need to turn off the auto gain
//! function
//**********************************************
CamStatus Arducam_SetISOSensitivity(Arducam* cam, int val);
//**********************************************
//!
//! @brief Set white balance mode
//!
//! @param  val `0` turn on automatic white balance `1` turn off automatic
//! white balance
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetAutoWhiteBalance(Arducam* cam, uint8_t val);
//**********************************************
//!
//! @brief  Set the white balance mode Manually
//!
//! @param   mode White balance mode
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetAutoWhiteBalanceMode(Arducam* cam, CAM_WHITE_BALANCE mode);
//**********************************************
//!
//! @brief Set special effects
//!
//! @param  effect Special effects mode
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetColorEffect(Arducam* cam, CAM_COLOR_FX effect);
//**********************************************
//!
//! @brief Set auto focus mode
//!
//! @param  val mode of autofocus
//!
//! @return Return operation status
//!
//! @note Only `5MP` cameras support auto focus control
//**********************************************
CamStatus Arducam_SetAutoFocus(Arducam* cam, uint8_t val);

//**********************************************
//!
//! @brief Get auto focus status
//!
//! @return Return 0x10：focus is finished
//!
//! @note Only `5MP` cameras support auto focus control
//**********************************************
uint8_t Arducam_GetAutoFocusSta(Arducam* cam);

//**********************************************
//!
//! @brief Set manual focus mode
//!
//! @param  value of VCM code
//!
//! @return Return operation status
//!
//! @note Only `5MP` cameras support maunal focus control
//**********************************************
CamStatus Arducam_SetManualFocus(Arducam* cam, uint16_t val);


//**********************************************
//!
//! @brief Set saturation level
//!
//! @param   level Saturation level
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetSaturation(Arducam* cam, CAM_STAURATION_LEVEL level);
//**********************************************
//!
//! @brief Set EV level
//!
//! @param  level EV level
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetEV(Arducam* cam, CAM_EV_LEVEL level);
//**********************************************
//!
//! @brief Set Contrast level
//!
//! @param  level Contrast level
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetContrast(Arducam* cam, CAM_CONTRAST_LEVEL level);
//**********************************************
//!
//! @brief Set Brightness level
//!
//! @param  level Brightness level
//!
//! @return Return operation status
//!
//**********************************************
CamStatus Arducam_SetBrightness(Arducam* cam, CAM_BRIGHTNESS_LEVEL level);
//**********************************************
//!
//! @brief Set Sharpness level
//!
//! @param  level Sharpness level
//!
//! @return Return operation status
//!
//! @note Only `3MP` cameras support sharpness control
//**********************************************
CamStatus Arducam_SetSharpness(Arducam* cam, CAM_SHARPNESS_LEVEL level);
//**********************************************
//!
//! @brief Set jpeg image quality
//!
//! @param  qualtiy Image Quality
//!
//! @return Return operation status
//**********************************************
CamStatus Arducam_SetImageQuality(Arducam* cam, IMAGE_QUALITY quality);
//**********************************************
//!
//! @brief Read image data with specified length to buffer
//!
//! @param  buff Buffer for storing camera data
//! @param  length The length of the available data to be read
//!
//! @return Returns the length actually read
//!
//! @note Transmission length should be less than `255`
//**********************************************
uint8_t Arducam_ReadBuff(Arducam* cam, uint8_t*, uint8_t);
//**********************************************
//!
//! @brief Read a byte from FIFO
//!
//! @return Returns Camera data
//!
//! @note Before calling this function, make sure that the data is available
//! in the buffer
//**********************************************
uint8_t Arducam_ReadByte(Arducam* cam);
//**********************************************
//!
//! @brief Debug mode
//!
//! @param  buff There are four bytes of buff Byte 1 indicates the device
//! address, Byte 2 indicates the high octet of the register, Byte 3
//! indicates the low octet of the register, and Byte 4 indicates the value
//! written to the register
//!
//**********************************************
void Arducam_DebugWriteRegister(Arducam* cam, uint8_t*);
//**********************************************
//!
//! @brief Create callback function
//!
//! @param  function Callback function name
//! @param  blockSize The length of the data transmitted by the callback
//! function at one time
//! @param  handle stop function Callback function name
//!
//! @note Transmission length should be less than `255`
//**********************************************
void Arducam_RegisterCallBack(Arducam* cam, BUFFER_CALLBACK, uint8_t, STOP_HANDLE);

//**********************************************
//!
//! @brief Turn on low power mode
//!
//**********************************************
void Arducam_LowPowerOn(Arducam* cam);

//**********************************************
//!
//! @brief Turn off low power mode
//!
//**********************************************
void Arducam_LowPowerOff(Arducam* cam);

//**********************************************
//!
//! @brief Get the length of the picture
//!
//! @return Return the length of the picture
//**********************************************
uint32_t Arducam_GetTotalLength(Arducam* cam);

//**********************************************
//!
//! @brief Get the length of the unread image
//!
//! @return Returns the length of the unread image
//**********************************************
uint32_t Arducam_GetReceivedLength(Arducam* cam);

//**********************************************
//!
//! @brief return a camera instance
//!
//! @return Return a ArducamCamera instance
//**********************************************
ArducamCamera* Arducam_GetCameraInstance(Arducam* cam);

#endif /* SRC_ARDUCAM_H_ */
