#include "data_logging.h"

/**
 * GLOBAL
 */
//FRESULT res; /* FatFs function common result code */
//uint32_t byteswritten, bytesread; /* File write/read counts */
//uint8_t wtext[] = "STM32 FATFS works great!"; /* File write buffer */
//uint8_t rtext[_MAX_SS];/* File read buffer */
//
//
//void SD_saveImage(uint8_t *imageBuff, size_t buffSize) {
//    char filename[32]; // Buffer to hold the file name
//    static uint32_t imageCount = 0; // Image count to generate unique filenames
//
//    // Generate a unique filename for the image
//    snprintf(filename, sizeof(filename), "image_%lu.jpg", imageCount++);
//
//    if (f_mount(&SDFatFS, "", 0) != FR_OK) {
//        printf("Failed to mount SD card\r\n");
//        Error_Handler();
//    } else {
//        // Open the file for writing
//        res = f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
//
//        if (res != FR_OK) {
//            printf("f_open failed: %d\r\n", res); // Print the error code
//            Error_Handler();
//        } else {
//            res = f_write(&SDFile, imageBuff, buffSize, (void *)&byteswritten);
//
//            if ((byteswritten == 0) || (res != FR_OK)) {
//                printf("Failed to write to file\r\n");
//                Error_Handler();
//            } else {
//                //printf("Image saved successfully as %s\r\n", filename);
//                f_close(&SDFile);
//            }
//        }
//    }
//
//    f_mount(&SDFatFS, (TCHAR const *)NULL, 0);
//}
//
//
//void SD_saveSensors(sensors_t *sensor_data){
//
//	char csv_line[256];
//
//	snprintf(csv_line, sizeof(csv_line),
//			 "%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f\n",
//			 sensor_data->filter_lat,
//			 sensor_data->filter_long,
//			 sensor_data->filter_alt,
//			 sensor_data->filter_vN,
//			 sensor_data->filter_vE,
//			 sensor_data->filter_vD,
//			 sensor_data->filter_q0,
//			 sensor_data->filter_q1,
//			 sensor_data->filter_q2,
//			 sensor_data->filter_q3,
//			 sensor_data->airspeed,
//			 sensor_data->time);
//
//	if(f_mount(&SDFatFS,"", 0) != FR_OK){
//		  printf("Failed to mount\r\n");
//		  Error_Handler();
//	} else{
//		//Open file for writing (Create)
//		FRESULT res = f_open(&SDFile, "STM32.CSV", FA_CREATE_ALWAYS | FA_WRITE);
//		if(res != FR_OK){
//			printf("f_open failed: %d\r\n", res);  // Print the error code
//			Error_Handler();
//		} else{
//			res = f_write(&SDFile, csv_line, strlen(csv_line), (void *)&byteswritten);
//			if((byteswritten == 0) || (res != FR_OK)){
//				printf("Failed to write to file\r\n");
//				Error_Handler();
//			} else{
//
//				f_close(&SDFile);
//			}
//		}
//	}
//
//	f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
//}
