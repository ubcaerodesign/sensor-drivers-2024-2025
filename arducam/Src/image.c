/* Includes ------------------------------------------------------------------*/
#include "image.h"
#include "file_check.h"
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define READ_IMAGE_LENGTH 255
#define BUFFER_SIZE       0xff

/* Private variables ---------------------------------------------------------*/
static uint32_t imageCounter = 0;
extern FIL SDFile;
extern FATFS SDFatFS;

// setup camera with preferred settings
void setupCamera(ArducamCamera* camera) {
	// camera config
    setSaturation(camera, CAM_STAURATION_LEVEL_MINUS_3);
    setColorEffect(camera, CAM_COLOR_FX_BLUEISH);

    // initialize the camera
    begin(camera);
    takePicture(camera, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);
}

// generate a unique img filename
void generateImgName(char* filename, size_t size) {
    FRESULT res;
    char tempFilename[32];

    do {
        snprintf(tempFilename, sizeof(tempFilename), "IMG_%04lu.jpg", imageCounter++);
        res = fileExists(tempFilename);
    } while (res == FR_OK);

    strncpy(filename, tempFilename, size);
}

// take a picture and save it to SD card
void cameraCaptureAndSaveImage(ArducamCamera* camera) {
    uint8_t imageBuff[BUFFER_SIZE]; // buffer for image data
    char uniqueFilename[32];
    FRESULT res;
    uint8_t imageData = 0;
    uint8_t imageDataNext = 0;
    uint8_t headFlag = 0;
    unsigned int i = 0;
    static long unsigned int count = 0;

    printf("Take a Picture...\r\n");

    // generate unique filename
    generateImgName(uniqueFilename, sizeof(uniqueFilename));
    printf("Saving as: %s\r\n", uniqueFilename);

    // take the pic
    takePicture(camera, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);

    printf("length is: %ld\r\n", camera->receivedLength);

    headFlag = 0;
    i = 0;

    while (camera->receivedLength) {
        imageData = imageDataNext;
        imageDataNext = readByte(camera);

        if (headFlag == 1) {
            imageBuff[i++] = imageDataNext;
            if (i >= BUFFER_SIZE) {
                res = f_write(&SDFile, imageBuff, i, NULL);
                if (res != FR_OK) {
                    printf("SD Write Error: %d\r\n", res);
                    break;
                }
                i = 0;
            }
        }

        if (imageData == 0xff && imageDataNext == 0xd8) {
            printf("Found JPEG header\r\n");

            f_mount(&SDFatFS, "", 0);
            res = f_open(&SDFile, uniqueFilename, FA_CREATE_ALWAYS | FA_WRITE);
            if (res != FR_OK) {
                printf("File open failed: %d\r\n", res);
                break;
            }

            headFlag = 1;
            count++;
            imageBuff[i++] = imageData;
            imageBuff[i++] = imageDataNext;
        }

        if (imageData == 0xff && imageDataNext == 0xd9) {
            printf("Found JPEG end marker\r\n");
            res = f_write(&SDFile, imageBuff, i, NULL);
            f_close(&SDFile);

            headFlag = 0;
            i = 0;
            break;
        }
    }

    printf("DONE, saved to SD card...\r\n");
}
