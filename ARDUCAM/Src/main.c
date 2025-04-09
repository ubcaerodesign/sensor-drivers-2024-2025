/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "data_logging.h"
#include "ArducamCamera.h"
#include <time.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define FRAMES_NUM        200     // # of frames
#define AVIOFFSET         240     // AVI header size
#define FRAME_RATE        10      // fps
#define BUFFER_SIZE       0xff    // img buff size

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SD_HandleTypeDef hsd2;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef huart8;

/* USER CODE BEGIN PV */

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

ArducamCamera myCAM;
int CS = 15;
#define READ_IMAGE_LENGTH           255

#define  BUFFER_SIZE  0xff
long unsigned int count = 0;
char name[10] = {0};
uint8_t rtLength = 0;
uint8_t imageData = 0;
uint8_t imageDataNext = 0;
uint8_t headFlag = 0;
unsigned int i = 0;
uint8_t imageBuff[BUFFER_SIZE] = {0};
uint8_t keyState = 0;
uint8_t isCaptureFlag = 0;

uint32_t imageCounter = 0;
uint32_t lastCaptureTime = 0;
uint32_t captureInterval = 5000;
char filename[32];
FRESULT res;
//uint32_t byteswritten, bytesread; /* File write/read counts */
//uint8_t wtext[] = "STM32 FATFS works great!"; /* File write buffer */
//uint8_t rtext[_MAX_SS];/* File read buffer */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_UART8_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void print_quartet(uint32_t i, FIL* fd);
FRESULT fileExists(const TCHAR* path);
void generateVideoFilename(char* filename, size_t size);
void generateImgName(char* filename, size_t size);
void setupCamera(ArducamCamera* camera);
FRESULT cameraRecordVideo(ArducamCamera* camera, uint16_t numFrames, CAM_IMAGE_MODE resolution);
void cameraCaptureAndSaveImage(ArducamCamera* camera);
void cameraCaptureAndSaveImageDMA(ArducamCamera* camera);
FRESULT cameraRecordVideoDMA(ArducamCamera* camera, uint16_t numFrames, CAM_IMAGE_MODE resolution);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print_quartet(uint32_t i, FIL* fd) {
  uint8_t buf[4];
  buf[0] = i & 0xFF;
  buf[1] = (i >> 8) & 0xFF;
  buf[2] = (i >> 16) & 0xFF;
  buf[3] = (i >> 24) & 0xFF;
  f_write(fd, buf, 4, NULL);
}

// check if  file exists
FRESULT fileExists(const TCHAR* path) {
    FRESULT res;
    FIL file;

    res = f_open(&file, path, FA_READ);
    if (res == FR_OK) {
        f_close(&file);
        return FR_OK; // file  exists
    }

    return res; // file doesn't exist or other error
}

void generateVideoFilename(char* filename, size_t size) {
    static uint32_t videoCounter = 0;
    FRESULT res;
    char tempFilename[32];

    do {
        snprintf(tempFilename, sizeof(tempFilename), "VID_%04lu.avi", videoCounter++);
        res = fileExists(tempFilename);
    } while (res == FR_OK);

    strncpy(filename, tempFilename, size);
}

// Generate a unique filename for the image
void generateImgName(char* filename, size_t size) {
    FRESULT res;
    char tempFilename[32];

    do {
        snprintf(tempFilename, sizeof(tempFilename), "IMG_%04lu.jpg", imageCounter++);
        res = fileExists(tempFilename);
    } while (res == FR_OK);

    strncpy(filename, tempFilename, size);
}

void setupCamera(ArducamCamera* camera) {
    // Set camera configurations directly in code
    // For example, setting resolution and format
    //camera->currentPictureMode = CAM_IMAGE_MODE_VGA;  // Example resolution, adjust as needed
    //camera->currentPixelFormat = CAM_IMAGE_PIX_FMT_JPG;  // Set pixel format (JPEG, RGB, etc.)

    // Set other configurations as needed, such as brightness, contrast, etc.
    //setBrightness(camera, CAM_BRIGHTNESS_LEVEL_1 );  // Example brightness level
    //setContrast(camera, CAM_CONTRAST_LEVEL_DEFAULT );      // Example contrast level
    setSaturation(camera, CAM_STAURATION_LEVEL_MINUS_3);  // Example saturation level
    setColorEffect(camera, CAM_COLOR_FX_BLUEISH);
    //setSharpness(camera, CAM_SHARPNESS_LEVEL_AUTO);    // Example sharpness level


    // Initialize the camera
    begin(camera);
    takePicture(camera, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);
}

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

    // Limit frames to maximum
    if (numFrames > FRAMES_NUM) {
        numFrames = FRAMES_NUM;
    }

    // Generate unique filename for the video
    generateVideoFilename(videoFilename, sizeof(videoFilename));
    printf("Recording video as: %s\r\n", videoFilename);

    // Mount the SD card and open the file
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

    // Start the multi-picture capture process
    takeMultiPictures(camera, resolution, CAM_IMAGE_PIX_FMT_JPG, numFrames);

    printf("Capturing frames...\r\n");

    // process frames (pre similar to what was done for taking imgs)
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

        if (imageData == 0xFF && imageDataNext == 0xD8) {
            headFlag = 1;
            jpeg_size = 0;

            // "00dc" indicates video data chunk
            f_write(&aviFile, "00dc", 4, NULL);

            // Placeholder for chunk size (will update later)
            f_write(&aviFile, zero_buf, 4, NULL);

            // jpeg data bneginning
            f_write(&aviFile, &imageData, 1, NULL);
            f_write(&aviFile, &imageDataNext, 1, NULL);
            jpeg_size += 2;
        }

        // detect JPEG end marker (0xFF 0xD9)
        if (imageData == 0xFF && imageDataNext == 0xD9) {
            // Write remaining data in buffer
            f_write(&aviFile, imageBuff, imageCount, NULL);
            imageCount = 0;
            headFlag = 0;

            // padding needed to align on 4-byte boundary
            remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;
            jpeg_size = jpeg_size + remnant;
            movi_size = movi_size + jpeg_size;

            // add zero padding if needed
            if (remnant > 0) {
                f_write(&aviFile, zero_buf, remnant, NULL);
            }

            // update chunk size (from line 269)
            position = f_tell(&aviFile);
            f_lseek(&aviFile, position - 4 - jpeg_size);
            print_quartet(jpeg_size, &aviFile);

            position = f_tell(&aviFile);
            f_lseek(&aviFile, position + 6);
            f_write(&aviFile, "AVI1", 4, NULL);

            // return to end of file for next frame
            position = f_tell(&aviFile);
            f_lseek(&aviFile, position + jpeg_size - 10);

            // increment frame counter
            frame_cnt++;
        }
    }

    printf("Finished capturing. Updating AVI header...\r\n");

    // update AVI header with final information
    f_lseek(&aviFile, 4);
    print_quartet(movi_size + 12 * frame_cnt + 4, &aviFile);

    uint32_t us_per_frame = 1000000 / FRAME_RATE;
    f_lseek(&aviFile, 0x20);
    print_quartet(us_per_frame, &aviFile);

    uint32_t max_bytes_per_sec = (frame_cnt > 0) ? (movi_size * FRAME_RATE / frame_cnt) : 0;
    f_lseek(&aviFile, 0x24);
    print_quartet(max_bytes_per_sec, &aviFile);

    f_lseek(&aviFile, 0x30);
    print_quartet(frame_cnt, &aviFile); //frame count

    f_lseek(&aviFile, 0xe0);
    print_quartet(frame_cnt, &aviFile);

    f_lseek(&aviFile, 0xe8);
    print_quartet(movi_size, &aviFile);

    f_close(&aviFile);

    printf("Video recording complete. Saved %d frames.\r\n", frame_cnt);

    return FR_OK;
}

void cameraCaptureAndSaveImage(ArducamCamera* camera) {
    uint8_t imageBuff[READ_IMAGE_LENGTH]; // Buffer to store image data
    char uniqueFilename[32];

    printf("Take a Picture...\r\n");

    // generate unique fiename
    generateImgName(uniqueFilename, sizeof(uniqueFilename));
    printf("Saving as: %s\r\n", uniqueFilename);

    // Capture the image (start the process)
   // takePicture(camera, (CAM_IMAGE_MODE)camera->currentPictureMode, (CAM_IMAGE_PIX_FMT)camera->currentPixelFormat);
    takePicture(camera, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);

    printf("length is: %ld\r\n", camera->receivedLength);

    headFlag = 0;
    i = 0;

    while (camera->receivedLength)
	{

		imageData = imageDataNext;
		imageDataNext = readByte(camera);

		if (headFlag == 1)
		{
			imageBuff[i++]=imageDataNext;
			if (i >= BUFFER_SIZE)
			{
				res = f_write(&SDFile, imageBuff, i, NULL);
				if (res != FR_OK) {
					printf("SD Write Error: %d\r\n", res);
					break;
				}
				i = 0;
			}
		}
		if (imageData == 0xff && imageDataNext ==0xd8)
		{
			printf("this is the head I think\r\n");

			snprintf(filename, sizeof(filename), "%lu.jpg", count++);

			f_mount(&SDFatFS, "", 0);
			res = f_open(&SDFile, uniqueFilename, FA_CREATE_ALWAYS | FA_WRITE);
			if (res != FR_OK) {
				printf("File open failed: %d\r\n", res);
				break;
			}

			headFlag = 1;
			count++;
			imageBuff[i++]=imageData;
			imageBuff[i++]=imageDataNext;
		}
		if (imageData == 0xff && imageDataNext ==0xd9)
		{
			printf("this is the end I think\r\n");
			res = f_write(&SDFile, imageBuff, i, NULL);
			f_close(&SDFile);

			headFlag = 0;
			i = 0;
			break;
		}
	}
    printf("DONE, saved to SD card..\r\n");
}

/**
 * @brief Captures an image and saves it to SD card using DMA for efficient data transfer
 *
 * @param camera ArducamCamera instance
 */
void cameraCaptureAndSaveImageDMA(ArducamCamera* camera) {
    uint8_t imageBuff[READ_IMAGE_LENGTH]; // Buffer to store image data
    char uniqueFilename[32];
    uint8_t headerDetectBuff[2] = {0, 0}; // Buffer to detect JPEG markers
    uint32_t bytesToRead = 0;
    uint32_t bufferOffset = 0;
    bool jpegStarted = false;
    bool jpegEnded = false;
    FIL jpegFile;
    FRESULT fileResult;
    UINT bytesWritten;

    printf("Taking picture with DMA...\r\n");

    // Generate unique filename
    generateImgName(uniqueFilename, sizeof(uniqueFilename));
    printf("Saving as: %s\r\n", uniqueFilename);

    // Capture the image
    takePicture(camera, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);
    printf("Image capture complete, size: %ld bytes\r\n", camera->receivedLength);

    // Mount the filesystem
    if (f_mount(&SDFatFS, "", 0) != FR_OK) {
        printf("Failed to mount SD card\r\n");
        return;
    }

    // Use a state machine approach for processing the JPEG data
    while (camera->receivedLength > 0 && !jpegEnded) {
        // Read data in chunks using DMA for efficiency
        bytesToRead = (camera->receivedLength > READ_IMAGE_LENGTH) ?
                       READ_IMAGE_LENGTH : camera->receivedLength;

        if (bytesToRead > 0) {
            uint32_t actualRead = readBuffDMA(camera, imageBuff, bytesToRead);

            if (actualRead == 0) {
                printf("Error reading from camera\r\n");
                break;
            }

            // Process the buffer to detect JPEG markers
            for (uint32_t i = 0; i < actualRead; i++) {
                // Shift buffer to track potential JPEG markers
                headerDetectBuff[0] = headerDetectBuff[1];
                headerDetectBuff[1] = imageBuff[i];

                // Check for JPEG start marker (0xFF 0xD8)
                if (!jpegStarted && headerDetectBuff[0] == 0xFF && headerDetectBuff[1] == 0xD8) {
                    printf("Found JPEG header\r\n");
                    jpegStarted = true;

                    // Open file for writing
                    fileResult = f_open(&jpegFile, uniqueFilename, FA_CREATE_ALWAYS | FA_WRITE);
                    if (fileResult != FR_OK) {
                        printf("Failed to create file: %d\r\n", fileResult);
                        return;
                    }

                    // Write the JPEG header (0xFF 0xD8)
                    uint8_t jpegHeader[2] = {0xFF, 0xD8};
                    f_write(&jpegFile, jpegHeader, 2, &bytesWritten);

                    // Initialize buffer for new data
                    bufferOffset = 0;
                }
                // Check for JPEG end marker (0xFF 0xD9)
                else if (jpegStarted && headerDetectBuff[0] == 0xFF && headerDetectBuff[1] == 0xD9) {
                    printf("Found JPEG end marker\r\n");
                    jpegEnded = true;

                    // Write accumulated data up to end marker
                    if (bufferOffset > 0) {
                        f_write(&jpegFile, imageBuff + i - bufferOffset, bufferOffset, &bytesWritten);
                    }

                    // Write the end marker itself
                    uint8_t jpegFooter[2] = {0xFF, 0xD9};
                    f_write(&jpegFile, jpegFooter, 2, &bytesWritten);

                    // Close the file
                    f_close(&jpegFile);
                    break;
                }
                // If we've started the JPEG but not yet found the end
                else if (jpegStarted) {
                    // Accumulate data in our buffer
                    bufferOffset++;

                    // If our buffer is full, write it to file
                    if (bufferOffset >= READ_IMAGE_LENGTH - 2) { // -2 to keep space for marker detection
                        f_write(&jpegFile, imageBuff + i - bufferOffset + 1, bufferOffset, &bytesWritten);
                        bufferOffset = 0;
                    }
                }
            }

            // If we have accumulated data and didn't just end the JPEG, write it
            if (jpegStarted && !jpegEnded && bufferOffset > 0) {
                f_write(&jpegFile, imageBuff + actualRead - bufferOffset, bufferOffset, &bytesWritten);
                bufferOffset = 0;
            }
        }
    }

    // Clean up if we didn't properly finish
    if (jpegStarted && !jpegEnded) {
        f_close(&jpegFile);
        printf("Warning: JPEG end marker not found\r\n");
    }

    printf("Image capture and save complete\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDMMC2_SD_Init();
  MX_UART8_Init();
  MX_FATFS_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  //uint8_t imageBuff[BUFFER_SIZE_test ]; // Create an image buffer
  //size_t buffSize = BUFFER_SIZE_test;
  //generateRandomImageBuffer(imageBuff, buffSize);
  //SD_saveImage(imageBuff, buffSize);

  /*uint8_t writeData = 0x33;
  uint8_t readData = 0x00;

  Camera_WriteRegister(0x00, writeData); // Write 0x55 to TEST_REG (0x00)
  HAL_Delay(10); // Wait for the camera to process the write
  readData = Camera_ReadRegister(0x00);
  printf("Written: 0x%02X, Read: 0x%02X\r\n", writeData, readData);

  if (readData == writeData) {
      printf("TEST_REG write-read verification successful!\r\n");
  } else {
      printf("TEST_REG write-read verification failed!\r\n");
  }

  writeData = 0x44;
  Camera_WriteRegister(0x00, writeData); // Write 0x55 to TEST_REG (0x00)
	HAL_Delay(10); // Wait for the camera to process the write
	readData = Camera_ReadRegister(0x00);
	printf("Written: 0x%02X, Read: 0x%02X\r\n", writeData, readData);

	if (readData == writeData) {
		printf("TEST_REG write-read verification successful!\r\n");
	} else {
		printf("TEST_REG write-read verification failed!\r\n");
	}*/

  printf("Initializing Arducam...\r\n");
  myCAM = createArducamCamera(CS);  // Create camera object with CS pin
  setupCamera(&myCAM);

  HAL_Delay(100);

  uint8_t test = cameraHeartBeat(&myCAM);
  printf("camera heartbeat: %d\r\n", test);

  // Capture a single picture
  cameraCaptureAndSaveImage(&myCAM);

  if(f_mount(&SDFatFS, "", 1) != FR_OK) {
      printf("SD card mount failed\r\n");
      Error_Handler();
  } else {
      printf("SD card mounted successfully\r\n");
  }

  for (int i = 0; i < 10; i++) {
	  cameraCaptureAndSaveImage(&myCAM);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART8 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart8, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
