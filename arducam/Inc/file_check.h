/*
 * file_checking.h
 *
 *  Created on: Mar 16, 2025
 *      Author: ryanz
 */

#ifndef INC_FILE_CHECK_H_
#define INC_FILE_CHECK_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "fatfs.h"

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Check if a file exists on the SD card
  * @param  path: Path to the file
  * @retval FRESULT: FR_OK if file exists, other code if not
  */
FRESULT fileExists(const TCHAR* path);

#ifdef __cplusplus
}
#endif

#endif /* INC_FILE_CHECK_H_ */
