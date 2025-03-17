/* Includes ------------------------------------------------------------------*/
#include "video.h"
#include "file_check.h"
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE       0xff    // Buffer size for image data

/* Private variables ---------------------------------------------------------*/
static uint32_t videoCounter = 0;
extern FATFS SDFatFS;

// AVI header structure
const uint8_t avi_header[AVIOFFSET] = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x40, 0x01, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, FRAME_RATE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54,
  0x10, 0x00, 0x00, 0x00, 0x6F, 0x64, 0x6D, 0x6C, 0x64, 0x6D, 0x6C, 0x68, 0x04, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};

// zero padding buffer
const uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};

// write a 32-bit int to file in little-endian format
void print_quartet(uint32_t i, FIL* fd) {
  uint8_t buf[4];
  buf[0] = i & 0xFF;
  buf[1] = (i >> 8) & 0xFF;
  buf[2] = (i >> 16) & 0xFF;
  buf[3] = (i >> 24) & 0xFF;
  f_write(fd, buf, 4, NULL);
}

// generate a unique filename for videos
void generateVideoFilename(char* filename, size_t size) {
    FRESULT res;
    char tempFilename[32];

    do {
        snprintf(tempFilename, sizeof(tempFilename), "VID_%04lu.avi", videoCounter++);
        res = fileExists(tempFilename);
    } while (res == FR_OK);

    strncpy(filename, tempFilename, size);
}

// record video and save as AVI file
FRESULT cameraRecordVideo(ArducamCamera* camera, uint16_t numFrames, CAM_IMAGE_MODE resolution) {
    uint8_t imageBuff[BUFFER_SIZE] = {0};  // Buffer for image data
    char videoFilename[32];                // Buffer for file name
    FRESULT res;                           // File operation result
    FIL aviFile;                           // File object
    uint32_t movi_size = 0;                // Size of movie data
    uint16_t frame_cnt = 0;                // Frame counter
    uint32_t jpeg_size = 0;                // Size of current JPEG frame
    uint8_t remnant = 0;                   // Padding bytes needed
    uint32_t position = 0;                 // File position
    uint8_t headFlag = 0;                  // Flag to indicate JPEG header found
    uint8_t imageData = 0;                 // Current byte of image data
    uint8_t imageDataNext = 0;             // Next byte of image data
    uint32_t imageCount = 0;               // Counter for image buffer

    // limit frames to maximum
    if (numFrames > FRAMES_NUM) {
        numFrames = FRAMES_NUM;
    }

    // Generate unique filename for the video
    generateVideoFilename(videoFilename, sizeof(videoFilename));
    printf("Recording video as: %s\r\n", videoFilename);

    if (f_mount(&SDFatFS, "", 0) != FR_OK) {
        printf("SD Card mount failed\r\n");
        return FR_DISK_ERR;
    }

    res = f_open(&aviFile, videoFilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("File open failed: %d\r\n", res);
        return res;
    }

    printf("File opened. Writing AVI header...\r\n");

    // Write AVI header
    for (int i = 0; i < AVIOFFSET; i++) {
        uint8_t headerByte = avi_header[i];
        f_write(&aviFile, &headerByte, 1, NULL);
    }

    printf("Starting video capture...\r\n");

    takeMultiPictures(camera, resolution, CAM_IMAGE_PIX_FMT_JPG, numFrames);

    printf("Capturing frames...\r\n");

    // [rocess  captured frames
    while (camera->receivedLength) {
        imageData = imageDataNext;
        imageDataNext = readByte(camera);

        if (headFlag == 1) {
            imageBuff[imageCount++] = imageDataNext;
            jpeg_size++;

            if (imageCount >= BUFFER_SIZE) {
                f_write(&aviFile, imageBuff, imageCount, NULL);
                imageCount = 0;
            }
        }

        // detect JPEG start marker (0xFF 0xD8)
        if (imageData == 0xFF && imageDataNext == 0xD8) {
            headFlag = 1;
            jpeg_size = 0;

            // write "00dc" marker (indicates video data chunk)
            f_write(&aviFile, "00dc", 4, NULL);
            // placeholder for chunk size (will update later)
            f_write(&aviFile, zero_buf, 4, NULL);
            // Start of JPEG data
            f_write(&aviFile, &imageData, 1, NULL);
            f_write(&aviFile, &imageDataNext, 1, NULL);
            jpeg_size += 2;
        }

        // setect JPEG end marker (0xFF 0xD9)
        if (imageData == 0xFF && imageDataNext == 0xD9) {
            // write remaining data in buffer
            f_write(&aviFile, imageBuff, imageCount, NULL);
            imageCount = 0;
            headFlag = 0;

            remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;
            jpeg_size = jpeg_size + remnant;
            movi_size = movi_size + jpeg_size;

            if (remnant > 0) {
                f_write(&aviFile, zero_buf, remnant, NULL);
            }

            // update chunk size
            position = f_tell(&aviFile);
            f_lseek(&aviFile, position - 4 - jpeg_size);
            print_quartet(jpeg_size, &aviFile);

            // write "AVI1" after chunk size
            position = f_tell(&aviFile);
            f_lseek(&aviFile, position + 6);
            f_write(&aviFile, "AVI1", 4, NULL);

            // return to end of file for next frame
            position = f_tell(&aviFile);
            f_lseek(&aviFile, position + jpeg_size - 10);


            frame_cnt++;
        }
    }

    printf("Finished capturing. Updating AVI header...\r\n");

    // update AVI header with final information (got chatgpt to add the header for the steps)
    // 1. Update RIFF file size
    f_lseek(&aviFile, 4);
    print_quartet(movi_size + 12 * frame_cnt + 4, &aviFile);

    // 2. Update frame timing
    uint32_t us_per_frame = 1000000 / FRAME_RATE;
    f_lseek(&aviFile, 0x20);
    print_quartet(us_per_frame, &aviFile);

    // 3. Update max bytes per second
    uint32_t max_bytes_per_sec = (frame_cnt > 0) ? (movi_size * FRAME_RATE / frame_cnt) : 0;
    f_lseek(&aviFile, 0x24);
    print_quartet(max_bytes_per_sec, &aviFile);

    // 4. Update total frames
    f_lseek(&aviFile, 0x30);
    print_quartet(frame_cnt, &aviFile);

    // 5. Update frames in list_odml section
    f_lseek(&aviFile, 0xe0);
    print_quartet(frame_cnt, &aviFile);

    // 6. Update movi_size
    f_lseek(&aviFile, 0xe8);
    print_quartet(movi_size, &aviFile);

    // Close the file
    f_close(&aviFile);

    printf("Video recording complete. Saved %d frames.\r\n", frame_cnt);

    return FR_OK;
}
