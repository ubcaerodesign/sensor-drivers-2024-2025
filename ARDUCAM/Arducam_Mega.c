#include "Arducam_Mega.h"

ArducamCamera Arducam_Init(int CS) {
    return createArducamCamera(CS);
}

CamStatus Arducam_Reset(Arducam* cam) {
    return reset(&cam->cameraInfo);
}

CamStatus Arducam_Begin(Arducam* cam) {
    return begin(&cam->cameraInfo);
}

CamStatus Arducam_TakePicture(Arducam* cam, CAM_IMAGE_MODE mode, CAM_IMAGE_PIX_FMT pixel_format) {
    return takePicture(&cam->cameraInfo, mode, pixel_format);
}

CamStatus Arducam_TakeMultiPictures(Arducam* cam, CAM_IMAGE_MODE mode, CAM_IMAGE_PIX_FMT pixel_format, uint8_t num) {
    return takeMultiPictures(&cam->cameraInfo, mode, pixel_format, num);
}

CamStatus Arducam_StartPreview(Arducam* cam, CAM_VIDEO_MODE mode) {
    return startPreview(&cam->cameraInfo, mode);
}

void Arducam_CaptureThread(Arducam* cam) {
    captureThread(&cam->cameraInfo);
}

CamStatus Arducam_StopPreview(Arducam* cam) {
    return stopPreview(&cam->cameraInfo);
}

CamStatus Arducam_SetAutoExposure(Arducam* cam, uint8_t val) {
    return setAutoExposure(&cam->cameraInfo, val);
}

CamStatus Arducam_SetAbsoluteExposure(Arducam* cam, uint32_t val) {
    return setAbsoluteExposure(&cam->cameraInfo, val);
}

CamStatus Arducam_SetAutoISOSensitive(Arducam* cam, uint8_t val) {
    return setAutoISOSensitive(&cam->cameraInfo, val);
}

CamStatus Arducam_SetISOSensitivity(Arducam* cam, int val) {
    return setISOSensitivity(&cam->cameraInfo, val);
}


CamStatus Arducam_SetAutoWhiteBalance(Arducam* cam, uint8_t val) {
    return setAutoWhiteBalance(&cam->cameraInfo, val);
}

CamStatus Arducam_SetAutoWhiteBalanceMode(Arducam* cam, CAM_WHITE_BALANCE mode) {
    return setAutoWhiteBalanceMode(&cam->cameraInfo, mode);
}

CamStatus Arducam_SetColorEffect(Arducam* cam, CAM_COLOR_FX effect) {
    return setColorEffect(&cam->cameraInfo, effect);
}

CamStatus Arducam_SetAutoFocus(Arducam* cam, uint8_t val) {
    return setAutoFocus(&cam->cameraInfo, val);
}

uint8_t Arducam_GetAutoFocusSta(Arducam* cam) {
    return getAutoFocusSta(&cam->cameraInfo);
}

CamStatus Arducam_SetManualFocus(Arducam* cam, uint16_t val) {
    return setManualFocus(&cam->cameraInfo, val);
}

CamStatus Arducam_SetSaturation(Arducam* cam, CAM_STAURATION_LEVEL level) {
    return setSaturation(&cam->cameraInfo, level);
}

CamStatus Arducam_SetEV(Arducam* cam, CAM_EV_LEVEL level) {
    return setEV(&cam->cameraInfo, level);
}

CamStatus Arducam_SetContrast(Arducam* cam, CAM_CONTRAST_LEVEL level) {
    return setContrast(&cam->cameraInfo, level);
}

CamStatus Arducam_SetBrightness(Arducam* cam, CAM_BRIGHTNESS_LEVEL level) {
    return setBrightness(&cam->cameraInfo, level);
}

CamStatus Arducam_SetSharpness(Arducam* cam, CAM_SHARPNESS_LEVEL level) {
    return setSharpness(&cam->cameraInfo, level);
}

CamStatus Arducam_SetImageQuality(Arducam* cam, IMAGE_QUALITY quality) {
    return setImageQuality(&cam->cameraInfo, quality);
}

uint8_t Arducam_ReadBuff(Arducam* cam, uint8_t* buff, uint8_t length) {
    return readBuff(&cam->cameraInfo, buff, length);
}

uint8_t Arducam_ReadByte(Arducam* cam) {
    return readByte(&cam->cameraInfo);
}

void Arducam_DebugWriteRegister(Arducam* cam, uint8_t* buff) {
    debugWriteRegister(&cam->cameraInfo, buff);
}

void Arducam_RegisterCallBack(Arducam* cam, BUFFER_CALLBACK function, uint8_t blockSize, STOP_HANDLE handle) {
    registerCallback(&cam->cameraInfo, function, blockSize, handle);
}

void Arducam_LowPowerOn(Arducam* cam) {
    lowPowerOn(&cam->cameraInfo);
}

void Arducam_LowPowerOff(Arducam* cam) {
    lowPowerOff(&cam->cameraInfo);
}

uint32_t Arducam_GetTotalLength(Arducam* cam) {
    return cam->cameraInfo.totalLength;
}

uint32_t Arducam_GetReceivedLength(Arducam* cam) {
    return cam->cameraInfo.receivedLength;
}

ArducamCamera* Arducam_GetCameraInstance(Arducam* cam) {
    return &cam->cameraInfo;
}
